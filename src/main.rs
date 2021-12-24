mod physics;
use macroquad::prelude::*;
use physics::nphysics;
use physics::rapier2d;
use physics::rapier2d_joint;

#[macroquad::main("BasicShapes")]
async fn main() {
    rapier2d_joint::progress().await;
    //nphysics::progress().await;
    // Change the above line to test the other engine
    //rapier2d::progress().await;
}
