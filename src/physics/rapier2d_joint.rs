use std::time::Instant;
use macroquad::prelude::*;
use nphysics2d::joint::FreeJoint;
use rapier2d::{crossbeam, dynamics};
use rapier2d::na::{Isometry2, Point2, Vector2};
use rapier2d::parry::partitioning::IndexedData;
use rapier2d::prelude::*;

// This allows easily switching between f32 or f64 precision
type FloatType = f32;

static METERS_ROOM_HEIGHT: FloatType = 2.5;
static METERS_PLAYER_HEIGHT: FloatType = 0.25;
static METERS_SENSOR_MARGIN: FloatType = 0.04;
static PIXELS_SCREEN_HEIGHT: FloatType = 600.0;
static PIXELS_SCREEN_WIDTH: FloatType = 800.0;
static PIXELS_PER_METER: FloatType = PIXELS_SCREEN_HEIGHT / METERS_ROOM_HEIGHT;
// Room is 600 pixels tall
static METERS_PER_PIXEL: FloatType = METERS_ROOM_HEIGHT / PIXELS_SCREEN_HEIGHT;
static METERS_SCREEN_WIDTH: FloatType = PIXELS_SCREEN_WIDTH * METERS_PER_PIXEL;
static METERS_SCREEN_HEIGHT: FloatType = PIXELS_SCREEN_HEIGHT * METERS_PER_PIXEL;
static PIXELS_PLAYER_HEIGHT: FloatType = PIXELS_PER_METER * METERS_PLAYER_HEIGHT;

static JUMP_IMPULSE: FloatType = 0.2;
static MINIMUM_JUMP_VELOCITY: FloatType = 1.0;

static MOVE_FORCE: FloatType = 1.0;

pub async fn progress() {
    let mut static_objects: Vec<StaticRectangle> = vec![];
    let mut dynamic_objects: Vec<DynamicCircle> = vec![];

    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();
    let mut joint_set = JointSet::new();

    static_objects.push(StaticRectangle::new(0.0, 0.0,
                                             METERS_SCREEN_WIDTH, METERS_PLAYER_HEIGHT,
                                             GREEN, &mut rigid_body_set, &mut collider_set));
    static_objects.push(StaticRectangle::new(METERS_SCREEN_WIDTH, -METERS_PLAYER_HEIGHT * 2.0,
                                             METERS_SCREEN_WIDTH, METERS_PLAYER_HEIGHT,
                                             GREEN, &mut rigid_body_set, &mut collider_set));
    static_objects.push(StaticRectangle::new(-METERS_SCREEN_WIDTH, -METERS_PLAYER_HEIGHT * 2.0,
                                             METERS_SCREEN_WIDTH, METERS_PLAYER_HEIGHT,
                                             GREEN, &mut rigid_body_set, &mut collider_set));
    dynamic_objects.push(DynamicCircle::new(-METERS_PLAYER_HEIGHT * 2.0, METERS_PLAYER_HEIGHT * 10.0,
                                            METERS_PLAYER_HEIGHT / 2.0, DARKGREEN, &mut rigid_body_set, &mut collider_set));
    dynamic_objects.push(DynamicCircle::new(-METERS_PLAYER_HEIGHT * 4.0, METERS_PLAYER_HEIGHT * 5.0,
                                            METERS_PLAYER_HEIGHT / 2.0, DARKGREEN, &mut rigid_body_set, &mut collider_set));
    dynamic_objects.push(DynamicCircle::new(-METERS_PLAYER_HEIGHT * 6.0, METERS_PLAYER_HEIGHT * 5.0,
                                            METERS_PLAYER_HEIGHT / 2.0, DARKGREEN, &mut rigid_body_set, &mut collider_set));
    static_objects.push(StaticRectangle::new(-METERS_PLAYER_HEIGHT * 8.0, METERS_PLAYER_HEIGHT * 5.0,
                                             METERS_PLAYER_HEIGHT / 2.0, METERS_PLAYER_HEIGHT / 2.0,
                                             GREEN, &mut rigid_body_set, &mut collider_set));


    let player = DynamicRectangle::new(0.0, METERS_PLAYER_HEIGHT * 5.0,
                                       METERS_PLAYER_HEIGHT, METERS_PLAYER_HEIGHT, BLUE,
                                       &mut rigid_body_set, &mut collider_set);

    /*
    let floor_sensor_collider = ColliderBuilder::
    cuboid((METERS_PLAYER_HEIGHT - METERS_SENSOR_MARGIN * 2.0) / 2.0,
           (METERS_PLAYER_HEIGHT/4.0 + METERS_SENSOR_MARGIN) / 2.0)
        .translation(vector![0.0, -METERS_SENSOR_MARGIN / 2.0 + METERS_PLAYER_HEIGHT/8.0 - METERS_PLAYER_HEIGHT/2.0])
        .density(0.0)
        .sensor(true)
        .active_events(ActiveEvents::INTERSECTION_EVENTS)
        .build();
    let floor_sensor_collider_handle = collider_set.insert_with_parent(floor_sensor_collider, player.handle, &mut rigid_body_set);

     */
    let floor_sensor = SensorRectangle::new(
        0.0, -METERS_SENSOR_MARGIN / 2.0 + METERS_PLAYER_HEIGHT/8.0 - METERS_PLAYER_HEIGHT/2.0,
        METERS_PLAYER_HEIGHT - METERS_SENSOR_MARGIN * 2.0, METERS_PLAYER_HEIGHT/4.0 + METERS_SENSOR_MARGIN,
        PURPLE, player.handle, &mut rigid_body_set, &mut collider_set, &mut joint_set
    );

    //let wall_sensor_collider_handle = collider_set.insert_with_parent(wall_sensor_collider, player.handle, &mut rigid_body_set);
    let has_jump_available = true;
    let mut touching_floor_count = 0;
    let mut touching_wall_count = 0;

    /* Create other structures necessary for the simulation. */
    let gravity = vector![0.0, -9.81];
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = BroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut ccd_solver = CCDSolver::new();
    let physics_hooks = ();
    // Initialize the event collector.
    let (contact_send, contact_recv) = crossbeam::channel::unbounded();
    let (intersection_send, intersection_recv) = crossbeam::channel::unbounded();
    let event_handler = ChannelEventCollector::new(intersection_send, contact_send);

    loop {
        let touching_floor = touching_floor_count > 0;
        let touching_wall = touching_wall_count > 0;

        if is_key_down(KeyCode::Left) {
            player.applyForce(Vector2::new(-MOVE_FORCE, 0.0), &mut rigid_body_set);
        }
        if is_key_down(KeyCode::Right) {
            player.applyForce(Vector2::new(MOVE_FORCE, 0.0), &mut rigid_body_set);
        }
        let is_moving_into_wall =
            (is_key_down(KeyCode::Left) || is_key_down(KeyCode::Right)) && touching_wall;


        if is_key_down(KeyCode::Up) {
            for (collider1, collider2, intersecting) in narrow_phase.intersections_with(floor_sensor.collider_handle) {
                if intersecting {
                    println!("The colliders {:?} and {:?} are intersecting!", collider1, collider2);
                }
            }

            if touching_floor && player.getVelocityY(&rigid_body_set) <= MINIMUM_JUMP_VELOCITY {
                player.applyImpulse(Vector2::new(0.0, JUMP_IMPULSE), &mut rigid_body_set);
                println!("Applying jump impulse");
            }
        }

        let integration_parameters = IntegrationParameters {
            ..IntegrationParameters::default()
        };
        physics_pipeline.step(
            &gravity,
            &integration_parameters,
            &mut island_manager,
            &mut broad_phase,
            &mut narrow_phase,
            &mut rigid_body_set,
            &mut collider_set,
            &mut joint_set,
            &mut ccd_solver,
            &physics_hooks,
            &event_handler,
        );

        while let Ok(intersection_event) = intersection_recv.try_recv() {
            // Handle the intersection event.
            println!("Received intersection event: {:?}", intersection_event);
            if (intersection_event.collider1 == floor_sensor.collider_handle ||
                intersection_event.collider2 == floor_sensor.collider_handle) &&
                intersection_event.collider1 != player.collider_handle &&
                    intersection_event.collider2 != player.collider_handle {
                if intersection_event.intersecting {
                    touching_floor_count += 1;
                } else {
                    touching_floor_count -= 1;
                }
            }
            println!("Touching floor count: {}", touching_floor_count);
        }

        clear_background(RED);

        // Create a camera that automatically converts physics positions to draw positions
        set_camera(&get_physics_camera(player.getPosition(&rigid_body_set)));

        // Draw all rectangles
        for rectangle in &static_objects {
            rectangle.draw(&rigid_body_set);
        }

        // Draw all circles
        for circle in &dynamic_objects {
            circle.draw(&rigid_body_set);
        }

        // Draw player
        player.draw(&rigid_body_set);

        // Draw floor sensor
        floor_sensor.draw(&rigid_body_set);

        // Reset camera for drawing UI in pixel numbers
        set_default_camera();

        next_frame().await
    }
}

fn get_physics_camera(target: Vec2) -> Camera2D {
    let mut camera = Camera2D {
        zoom: vec2(1.0 / METERS_ROOM_HEIGHT as f32, 1.0 / (screen_height() / screen_width() * METERS_ROOM_HEIGHT as f32)),
        offset: vec2(0., 0.),
        target: vec2(target.x as f32, target.y as f32),
        rotation: 0.,
        ..Default::default()
    };
    camera
}

// Rectangle
struct StaticRectangle {
    handle: RigidBodyHandle,
    width: FloatType,
    height: FloatType,
    color: Color,
}

impl StaticRectangle {
    fn new(x: FloatType, y: FloatType, width: FloatType, height: FloatType, color: Color, rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet) -> Self {
        let mut rigid_body = RigidBodyBuilder::new_static()
            .translation(vector![x, y])
            .can_sleep(false)
            .build();
        let ground_collider = ColliderBuilder::cuboid(width / 2.0, height / 2.0)
            .restitution(0.5)
            .density(1.0)
            .friction(0.2)
            .active_events(ActiveEvents::INTERSECTION_EVENTS)
            .build();
        let rigid_body_handle = rigid_body_set.insert(rigid_body);
        collider_set.insert_with_parent(ground_collider, rigid_body_handle, rigid_body_set);

        Self {
            handle: rigid_body_handle,
            width,
            height,
            color,
        }
    }
    fn draw(&self, rigid_body_set: &RigidBodySet) {
        let body = &rigid_body_set[self.handle];
        draw_rectangle(
            (body.translation().x - (self.width / 2.0)) as f32,
            (body.translation().y - (self.height / 2.0)) as f32,
            self.width as f32,
            self.height as f32,
            self.color,
        );
    }
    fn getPosition(&self, rigid_body_set: &RigidBodySet) -> Vec2 {
        let body = &rigid_body_set[self.handle];
        return Vec2::new(body.translation().x as f32, body.translation().y as f32);
    }
}

// Circle
struct DynamicCircle {
    handle: RigidBodyHandle,
    radius: FloatType,
    color: Color,
}

impl DynamicCircle {
    fn new(x: FloatType, y: FloatType, radius: FloatType, color: Color, rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet) -> Self {
        let mut rigid_body = RigidBodyBuilder::new_dynamic()
            .translation(vector![x, y])
            .can_sleep(false)
            .build();
        rigid_body.enable_ccd(true);
        let ground_collider = ColliderBuilder::ball(radius)
            .restitution(0.963)
            .restitution_combine_rule(CoefficientCombineRule::Max)
            .active_events(ActiveEvents::INTERSECTION_EVENTS)
            .density(1.0)
            .build();
        let rigid_body_handle = rigid_body_set.insert(rigid_body);
        collider_set.insert_with_parent(ground_collider, rigid_body_handle, rigid_body_set);

        Self {
            handle: rigid_body_handle,
            radius,
            color,
        }
    }
    fn draw(&self, rigid_body_set: &RigidBodySet) {
        let body = &rigid_body_set[self.handle];
        draw_circle(
            body.translation().x as f32,
            body.translation().y as f32,
            self.radius as f32,
            self.color,
        );
    }
    fn getPosition(&self, rigid_body_set: &RigidBodySet) -> Vec2 {
        let body = &rigid_body_set[self.handle];
        return Vec2::new(body.translation().x as f32, body.translation().y as f32);
    }
    fn applyForce(&self, force: Vector2<FloatType>, rigid_body_set: &mut RigidBodySet) {
        let mut body = &mut rigid_body_set[self.handle];
        body.apply_force(force, true);
    }
}

// Rectangle
struct DynamicRectangle {
    handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    width: FloatType,
    height: FloatType,
    color: Color,
}

impl DynamicRectangle {
    fn new(x: FloatType, y: FloatType, width: FloatType, height: FloatType, color: Color, rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet) -> Self {
        let mut rigid_body = RigidBodyBuilder::new_dynamic()
            .translation(vector![x, y])
            .can_sleep(false)
            .lock_rotations()
            .build();
        rigid_body.enable_ccd(true);
        let ground_collider = ColliderBuilder::cuboid(width / 2.0, height / 2.0)
            .restitution(0.0)
            .restitution_combine_rule(CoefficientCombineRule::Min)
            .density(1.0)
            .active_events(ActiveEvents::INTERSECTION_EVENTS)
            .build();
        let rigid_body_handle = rigid_body_set.insert(rigid_body);
        let collider_handle = collider_set.insert_with_parent(ground_collider, rigid_body_handle, rigid_body_set);

        Self {
            handle: rigid_body_handle,
            collider_handle,
            width,
            height,
            color,
        }
    }
    fn draw(&self, rigid_body_set: &RigidBodySet) {
        let body = &rigid_body_set[self.handle];
        // Future: See if we can turn a polygon into a 2D triangle mesh and render that
        // Draw ground
        draw_rectangle(
            (body.translation().x - (self.width / 2.0)) as f32,
            (body.translation().y - (self.height / 2.0)) as f32,
            self.width as f32,
            self.height as f32,
            self.color,
        );
    }
    fn getPosition(&self, rigid_body_set: &RigidBodySet) -> Vec2 {
        let body = &rigid_body_set[self.handle];
        return Vec2::new(body.translation().x as f32, body.translation().y as f32);
    }
    fn applyForce(&self, force: Vector2<FloatType>, rigid_body_set: &mut RigidBodySet) {
        let mut body = &mut rigid_body_set[self.handle];
        body.apply_force(force, true);
    }
    fn applyImpulse(&self, impulse: Vector2<FloatType>, rigid_body_set: &mut RigidBodySet) {
        let mut body = &mut rigid_body_set[self.handle];
        body.apply_impulse(impulse, true);
    }
    fn getVelocityY(&self, rigid_body_set: &RigidBodySet) -> FloatType {
        let mut body = &rigid_body_set[self.handle];
        return body.linvel().y;
    }
}


// Sensor Rectangle
struct SensorRectangle {
    connectedHandle: RigidBodyHandle,
    handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    width: FloatType,
    height: FloatType,
    color: Color,
}

impl SensorRectangle {
    fn new(offsetX: FloatType, offsetY: FloatType, width: FloatType, height: FloatType, color: Color, connectedHandle: RigidBodyHandle, rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet, joint_set: &mut JointSet) -> Self {
        let connectedBody = &rigid_body_set[connectedHandle];
        let mut rigid_body = RigidBodyBuilder::new_dynamic()
            .translation(vector![connectedBody.translation().x + offsetX, connectedBody.translation().y + offsetY])
            .can_sleep(false)
            .lock_rotations()
            .build();
        rigid_body.enable_ccd(true);
        let sensor_collider = ColliderBuilder::cuboid(width / 2.0, height / 2.0)
            .restitution(0.0)
            .friction(0.0)
            .density(0.00000001)
            .sensor(true)
            .active_events(ActiveEvents::INTERSECTION_EVENTS)
            .build();
        let rigid_body_handle = rigid_body_set.insert(rigid_body);
        let collider_handle = collider_set.insert_with_parent(sensor_collider, rigid_body_handle, rigid_body_set);

        let joint = BallJoint::new( point![0.0, offsetY], Point::origin());
        //let joint = FixedJoint::new(Isometry2::translation(0.0, offsetY as Real), Isometry2::translation(0.0, 0.0));
        //let joint = FreeJoint::new(Isometry2::translation(0.0, 0.0));
        let joint_handle = joint_set.insert(connectedHandle, rigid_body_handle, joint);

        Self {
            connectedHandle: connectedHandle,
            handle: rigid_body_handle,
            collider_handle: collider_handle,
            width,
            height,
            color,
        }
    }
    fn draw(&self, rigid_body_set: &RigidBodySet) {
        let body = &rigid_body_set[self.handle];
        // Future: See if we can turn a polygon into a 2D triangle mesh and render that
        // Draw ground
        draw_rectangle(
            (body.translation().x - (self.width / 2.0)) as f32,
            (body.translation().y - (self.height / 2.0)) as f32,
            self.width as f32,
            self.height as f32,
            self.color,
        );
    }
    fn getPosition(&self, rigid_body_set: &RigidBodySet) -> Vec2 {
        let body = &rigid_body_set[self.handle];
        return Vec2::new(body.translation().x as f32, body.translation().y as f32);
    }
    fn applyForce(&self, force: Vector2<FloatType>, rigid_body_set: &mut RigidBodySet) {
        let mut body = &mut rigid_body_set[self.handle];
        body.apply_force(force, true);
    }
    fn applyImpulse(&self, impulse: Vector2<FloatType>, rigid_body_set: &mut RigidBodySet) {
        let mut body = &mut rigid_body_set[self.handle];
        body.apply_impulse(impulse, true);
    }
    fn getVelocityY(&self, rigid_body_set: &RigidBodySet) -> FloatType {
        let mut body = &rigid_body_set[self.handle];
        return body.linvel().y;
    }
}