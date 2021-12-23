use std::time::Instant;
use macroquad::prelude::*;
use nphysics2d::algebra::Force2;
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::material::{BasicMaterial, MaterialCombineMode, MaterialHandle};
use nphysics2d::nalgebra::{vector, Vector2};
use nphysics2d::object::{BodyPartHandle, ColliderDesc, DefaultBodyHandle, DefaultBodySet, DefaultColliderHandle, DefaultColliderSet};
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld, MechanicalWorld};
use nphysics2d::object::{BodyStatus, RigidBodyDesc};
use nphysics2d::math::{Velocity, Inertia, ForceType};
use nphysics2d::ncollide2d::query;
use nphysics2d::ncollide2d::query::Proximity;
use nphysics2d::ncollide2d::shape::{Ball, Cuboid, ShapeHandle};

// This allows easily switching between f32 or f64 precision
type FloatType = f32;

static METERS_ROOM_HEIGHT: FloatType = 2.5;
static METERS_PLAYER_HEIGHT: FloatType = 0.25;
static METERS_SENSOR_DETECT_MARGIN: FloatType = 0.1;
static METERS_SENSOR_SIDE_MARGIN: FloatType = 0.04;
static PIXELS_SCREEN_HEIGHT: FloatType = 600.0;
static PIXELS_SCREEN_WIDTH: FloatType = 800.0;
static PIXELS_PER_METER: FloatType = PIXELS_SCREEN_HEIGHT / METERS_ROOM_HEIGHT;
static METERS_PER_PIXEL: FloatType = METERS_ROOM_HEIGHT / PIXELS_SCREEN_HEIGHT;
static METERS_SCREEN_WIDTH: FloatType = PIXELS_SCREEN_WIDTH * METERS_PER_PIXEL;
static METERS_SCREEN_HEIGHT: FloatType = PIXELS_SCREEN_HEIGHT * METERS_PER_PIXEL;
static PIXELS_PLAYER_HEIGHT: FloatType = PIXELS_PER_METER * METERS_PLAYER_HEIGHT;

static JUMP_IMPULSE: FloatType = 3.0;
static MINIMUM_JUMP_VELOCITY: FloatType = 1.0;

static MOVE_FORCE: FloatType = 10.0;

pub async fn progress() {
    let mut static_objects: Vec<StaticRectangle> = vec![];
    let mut dynamic_objects: Vec<DynamicCircle> = vec![];

    let mut rigid_body_set = DefaultBodySet::new();
    let mut collider_set = DefaultColliderSet::new();

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

    let shape = ShapeHandle::new(Cuboid::new(Vector2::new((METERS_PLAYER_HEIGHT - METERS_SENSOR_SIDE_MARGIN * 2.0) / 2.0,
                                             (METERS_PLAYER_HEIGHT/4.0 + METERS_SENSOR_DETECT_MARGIN) / 2.0)));
    let sensor = ColliderDesc::new(shape)
        .translation(Vector2::new(0.0, -METERS_SENSOR_DETECT_MARGIN / 2.0 + METERS_PLAYER_HEIGHT/8.0 - METERS_PLAYER_HEIGHT/2.0))
        .sensor(true)
        .density(0.0)
        .build(BodyPartHandle(player.handle, 0));
    // Add to the collider set.
    let floor_sensor_collider_handle = collider_set.insert(sensor);
    let has_jump_available = true;
    let mut touching_floor_count = 0;
    let mut touching_wall_count = 0;

    /* Create other structures necessary for the simulation. */
    let mut mechanical_world: MechanicalWorld<f32, DefaultBodyHandle, DefaultColliderHandle> =
        DefaultMechanicalWorld::new(Vector2::new(0.0, -9.81));
    mechanical_world.set_timestep(1.0 / 60.0);
    let mut geometrical_world = DefaultGeometricalWorld::new();

    let mut joint_constraints = DefaultJointConstraintSet::new();
    let mut force_generators = DefaultForceGeneratorSet::new();

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
            println!("Vel Y: {}, touching floor count: {}", player.getVelocityY(&mut rigid_body_set), touching_floor_count);
            if touching_floor && player.getVelocityY(&mut rigid_body_set) <= MINIMUM_JUMP_VELOCITY {
                player.applyImpulse(Vector2::new(0.0, JUMP_IMPULSE), &mut rigid_body_set);
                println!("Applying jump impulse");
            }
        }

        // Run the simulation.
        mechanical_world.step(
            &mut geometrical_world,
            &mut rigid_body_set,
            &mut collider_set,
            &mut joint_constraints,
            &mut force_generators,
        );

        for proximity in geometrical_world.proximity_events() {
            println!("Received intersection event: {:?}", proximity);
            if proximity.collider1 == floor_sensor_collider_handle ||
                proximity.collider2 == floor_sensor_collider_handle {
                if proximity.new_status == query::Proximity::Intersecting && proximity.prev_status != query::Proximity::Intersecting {
                    touching_floor_count += 1;
                } else if proximity.new_status != query::Proximity::Intersecting && proximity.prev_status == query::Proximity::Intersecting {
                    touching_floor_count -= 1;
                }
            }
            println!("Touching floor count: {}", touching_floor_count);
        }

        clear_background(RED);

        // Create a camera that automatically converts physics positions to draw positions
        set_camera(&get_physics_camera(player.getPosition(&mut rigid_body_set)));

        // Draw all rectangles
        for rectangle in &static_objects {
            rectangle.draw(&mut rigid_body_set);
        }

        // Draw all circles
        for circle in &dynamic_objects {
            circle.draw(&mut rigid_body_set);
        }

        // Draw player
        player.draw(&mut rigid_body_set);

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
    handle: DefaultBodyHandle,
    width: FloatType,
    height: FloatType,
    color: Color,
}

impl StaticRectangle {
    fn new(x: FloatType, y: FloatType, width: FloatType, height: FloatType, color: Color, rigid_body_set: &mut DefaultBodySet<f32>, collider_set: &mut DefaultColliderSet<f32>) -> Self {
        let rigid_body = RigidBodyDesc::new()
            .translation(Vector2::new(x, y))
            .mass(1.0)
            .status(BodyStatus::Static)
            .build();
        let rigid_body_handle = rigid_body_set.insert(rigid_body);

        let shape = ShapeHandle::new(Cuboid::new(Vector2::new(width / 2.0, height / 2.0)));
        let collider = ColliderDesc::new(shape)
            .density(1.0)
            .material(MaterialHandle::new(BasicMaterial {
                id: None,
                restitution: 0.5,
                friction: 0.2,
                surface_velocity: None,
                restitution_combine_mode: MaterialCombineMode::Average,
                friction_combine_mode: MaterialCombineMode::Average,
            }))
            .build(BodyPartHandle(rigid_body_handle, 0));
        let collider_handle = collider_set.insert(collider);

        Self {
            handle: rigid_body_handle,
            width,
            height,
            color,
        }
    }
    fn draw(&self, rigid_body_set: &mut DefaultBodySet<f32>) {
        let body = rigid_body_set.get(self.handle).unwrap();
        // Future: See if we can turn a polygon into a 2D triangle mesh and render that
        // Draw ground
        draw_rectangle(
            (body.part(0).unwrap().position().translation.x - (self.width / 2.0)) as f32,
            (body.part(0).unwrap().position().translation.y - (self.height / 2.0)) as f32,
            self.width as f32,
            self.height as f32,
            self.color,
        );
    }
}

// Circle
struct DynamicCircle {
    handle: DefaultBodyHandle,
    radius: FloatType,
    color: Color,
}

impl DynamicCircle {
    fn new(x: FloatType, y: FloatType, radius: FloatType, color: Color, rigid_body_set: &mut DefaultBodySet<f32>, collider_set: &mut DefaultColliderSet<f32>) -> Self {
        let rigid_body = RigidBodyDesc::new()
            .translation(Vector2::new(x, y))
            .mass(1.0)
            .status(BodyStatus::Dynamic)
            .build();
        let rigid_body_handle = rigid_body_set.insert(rigid_body);

        let shape = ShapeHandle::new(Ball::new(radius));
        let collider = ColliderDesc::new(shape)
            .density(1.0)
            .material(MaterialHandle::new(BasicMaterial {
                id: None,
                restitution: 1.0,
                friction: 0.0,
                surface_velocity: None,
                restitution_combine_mode: MaterialCombineMode::Max,
                friction_combine_mode: MaterialCombineMode::Average,
            }))
            .build(BodyPartHandle(rigid_body_handle, 0));
        let collider_handle = collider_set.insert(collider);

        Self {
            handle: rigid_body_handle,
            radius,
            color,
        }
    }
    fn draw(&self, rigid_body_set: &mut DefaultBodySet<f32>) {
        let body = rigid_body_set.get(self.handle).unwrap();
        // Future: See if we can turn a polygon into a 2D triangle mesh and render that
        // Draw circle
        draw_circle(
            body.part(0).unwrap().position().translation.x as f32,
            body.part(0).unwrap().position().translation.y as f32,
            self.radius as f32,
            self.color,
        );
    }
}

// Rectangle
struct DynamicRectangle {
    handle: DefaultBodyHandle,
    width: FloatType,
    height: FloatType,
    color: Color,
}

impl DynamicRectangle {
    fn new(x: FloatType, y: FloatType, width: FloatType, height: FloatType, color: Color, rigid_body_set: &mut DefaultBodySet<f32>, collider_set: &mut DefaultColliderSet<f32>) -> Self {
        let rigid_body = RigidBodyDesc::new()
            .translation(Vector2::new(x, y))
            .status(BodyStatus::Dynamic)
            .mass(1.0)
            .build();

        let rigid_body_handle = rigid_body_set.insert(rigid_body);

        let shape = ShapeHandle::new(Cuboid::new(Vector2::new(width / 2.0, height / 2.0)));
        let collider = ColliderDesc::new(shape)
            .density(1.0)
            .material(MaterialHandle::new(BasicMaterial {
                id: None,
                restitution: 0.0,
                friction: 0.0,
                surface_velocity: None,
                restitution_combine_mode: MaterialCombineMode::Min,
                friction_combine_mode: MaterialCombineMode::Average,
            }))
            .build(BodyPartHandle(rigid_body_handle, 0));
        let collider_handle = collider_set.insert(collider);

        Self {
            handle: rigid_body_handle,
            width,
            height,
            color,
        }
    }
    fn draw(&self, rigid_body_set: &mut DefaultBodySet<f32>) {
        let body = &rigid_body_set.get(self.handle).unwrap();
        draw_rectangle(
            (body.part(0).unwrap().position().translation.x - (self.width / 2.0)) as f32,
            (body.part(0).unwrap().position().translation.y - (self.height / 2.0)) as f32,
            self.width as f32,
            self.height as f32,
            self.color,
        );
        // Draw floor sensor
        let x = body.part(0).unwrap().position().translation.x;
        let y = body.part(0).unwrap().position().translation.y;
        let width = METERS_PLAYER_HEIGHT - METERS_SENSOR_SIDE_MARGIN * 2.0;
        let height = METERS_PLAYER_HEIGHT/4.0 + METERS_SENSOR_DETECT_MARGIN;
        draw_rectangle(x-width / 2.0, y-height/2.0-METERS_SENSOR_DETECT_MARGIN / 2.0 + METERS_PLAYER_HEIGHT/8.0 - METERS_PLAYER_HEIGHT/2.0,
                       width, height, PURPLE);
    }

    fn getPosition(&self, rigid_body_set: &mut DefaultBodySet<f32>) -> Vec2 {
        let body = &rigid_body_set.get(self.handle).unwrap();
        return Vec2::new(body.part(0).unwrap().position().translation.x as f32,
                         body.part(0).unwrap().position().translation.y as f32);
    }
    fn applyForce(&self, force: Vector2<FloatType>, rigid_body_set: &mut DefaultBodySet<f32>) {
        let mut body = &mut rigid_body_set.get_mut(self.handle).unwrap();
        body.apply_force(0, &Force2::new(force, 0.0), ForceType::Force, true);
    }
    fn applyImpulse(&self, impulse: Vector2<FloatType>, rigid_body_set: &mut DefaultBodySet<f32>) {
        let mut body = &mut rigid_body_set.get_mut(self.handle).unwrap();
        body.apply_force(0, &Force2::new(impulse, 0.0), ForceType::Impulse, true);
    }
    fn getVelocityY(&self, rigid_body_set: &mut DefaultBodySet<f32>) -> FloatType {
        let body = &rigid_body_set.get(self.handle).unwrap();
        return body.part(0).unwrap().velocity().linear.y;
    }
}