use macroquad::prelude as mq;

use tiny_physics_engine::core::{Integrator, RigidBody, World};
use tiny_physics_engine::math::vec::Vec2;

#[cfg(not(feature = "visualize"))]
compile_error!("Enable this example with: cargo run --example slope_friction --features visualize");
#[cfg(feature = "visualize")]
use tiny_physics_engine::utils::visualize::draw_world;

#[macroquad::main("Tiny Physics — Slope Friction")]
async fn main() {
    let gravity = Vec2::new(0.0, -9.81);
    let mut world = World::new(gravity, Integrator::SemiImplicitEuler);
    world.solver.params.restitution = 0.0;
    world.solver.params.friction = 0.9;
    world.solver.iterations = 18;

    let scale = 70.0;
    let fixed_dt = 1.0 / 240.0;
    let mut accumulator = 0.0f32;

    // Static slope (mass=0) as a rotated box.
    let slope_angle = 0.35; // ~20°
    let slope = RigidBody::box_xy(Vec2::new(0.0, -2.8), slope_angle, 0.0, 12.0, 0.6);
    world.add(Box::new(slope));

    // Static floor (optional catch).
    let floor = RigidBody::box_xy(Vec2::new(0.0, -6.2), 0.0, 0.0, 30.0, 0.8);
    world.add(Box::new(floor));

    // Dynamic box placed slightly above the slope surface.
    let mut box1 = RigidBody::box_xy(Vec2::new(-3.0, 0.8), 0.05, 1.0, 1.0, 0.6);
    box1.vel = Vec2::new(0.0, 0.0);
    world.add(Box::new(box1));

    // A second box to show stacking/sliding behavior on the slope.
    let mut box2 = RigidBody::box_xy(Vec2::new(-4.2, 1.8), -0.08, 1.0, 0.9, 0.5);
    box2.vel = Vec2::new(0.0, 0.0);
    world.add(Box::new(box2));

    loop {
        let mut frame_dt = mq::get_frame_time();
        if frame_dt > 1.0 / 30.0 {
            frame_dt = 1.0 / 30.0;
        }
        accumulator += frame_dt;
        while accumulator >= fixed_dt {
            world.step(fixed_dt);
            accumulator -= fixed_dt;
        }

        draw_world(&world, scale);
        mq::next_frame().await;
    }
}
