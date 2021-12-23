mod physics;
use macroquad::prelude::*;
use physics::nphysics;
use physics::rapier2d;

#[macroquad::main("BasicShapes")]
async fn main() {
    nphysics::progress().await;
    // Change the above line to test the other engine
    //rapier2d::progress().await;
}
