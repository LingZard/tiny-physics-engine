use macroquad::prelude as mq;

use tiny_physics_engine::core::{Integrator, RigidBody, World};
use tiny_physics_engine::math::vec::Vec2;

#[cfg(not(feature = "visualize"))]
compile_error!("Enable this example with: cargo run --example box_box_spin --features visualize");
#[cfg(feature = "visualize")]
use tiny_physics_engine::utils::visualize::draw_world;

#[macroquad::main("Tiny Physics — Box/Box Spin Impact")]
async fn main() {
    // No gravity: isolate contact manifold + angular response.
    let gravity = Vec2::zero();
    let mut world = World::new(gravity, Integrator::SemiImplicitEuler);
    world.solver.params.restitution = 0.2;
    world.solver.params.friction = 0.6;
    world.solver.iterations = 16;

    let scale = 80.0;
    let fixed_dt = 1.0 / 240.0;
    let mut accumulator = 0.0f32;

    // A heavier spinning box moving right.
    let mut a = RigidBody::box_xy(Vec2::new(-4.0, 0.0), 0.35, 3.0, 1.6, 0.8);
    a.vel = Vec2::new(7.0, 0.0);
    a.omega = 6.0;
    world.add(Box::new(a));

    // A lighter box moving left.
    let mut b = RigidBody::box_xy(Vec2::new(4.0, 0.3), -0.2, 1.0, 1.2, 1.0);
    b.vel = Vec2::new(-5.5, 0.0);
    b.omega = -2.0;
    world.add(Box::new(b));

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
