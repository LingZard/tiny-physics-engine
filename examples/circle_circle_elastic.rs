use macroquad::prelude as mq;

use tiny_physics_engine::core::{Integrator, RigidBody, World};
use tiny_physics_engine::math::vec::Vec2;

#[cfg(not(feature = "visualize"))]
compile_error!(
    "Enable this example with: cargo run --example circle_circle_elastic --features visualize"
);
#[cfg(feature = "visualize")]
use tiny_physics_engine::utils::visualize::draw_world;

#[macroquad::main("Tiny Physics — Circle/Circle Elastic")]
async fn main() {
    // No gravity: isolate collision response.
    let gravity = Vec2::zero();
    let mut world = World::new(gravity, Integrator::SemiImplicitEuler);

    // Nearly elastic collision, no friction.
    world.solver.params.restitution = 1.0;
    world.solver.params.friction = 0.0;
    world.solver.iterations = 12;

    let scale = 80.0;
    let fixed_dt = 1.0 / 240.0;
    let mut accumulator = 0.0f32;

    let r = 0.45;
    let m = 1.0;

    let mut a = RigidBody::circle(Vec2::new(-3.0, 0.0), 0.0, m, r);
    a.vel = Vec2::new(6.0, 0.5);
    world.add(Box::new(a));

    let mut b = RigidBody::circle(Vec2::new(3.0, 0.2), 0.0, m, r);
    b.vel = Vec2::new(-6.0, -0.3);
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
