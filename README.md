# Sensor-Pain
This is my painful experience trying to use sensors in Rapier2D and NPhysics. We're using a sensor on the player box to detect if the player can jump. Unfortunately the sensor appears to stop working after the player box gets hit on the head by a bouncing ball. In NPhysics, the right platform nevers gets detected as floor.  
It is likely that this isn't a problem with Rapier2D or NPhysics but user error. For example, the sensor could be placed incorrectly and the rendered visual of the sensor is inaccurate. That is difficult to diagnose.  

Run using: `cargo run --package Sensor-Pain --bin Sensor-Pain --release`
