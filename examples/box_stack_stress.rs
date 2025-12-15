use macroquad::prelude as mq;

use tiny_physics_engine::core::{Integrator, RigidBody, World};
use tiny_physics_engine::math::vec::Vec2;

#[cfg(not(feature = "visualize"))]
compile_error!(
    "Enable this example with: cargo run --example box_stack_stress --features visualize"
);
#[cfg(feature = "visualize")]
use tiny_physics_engine::utils::visualize::draw_world;

#[macroquad::main("Tiny Physics — Box Stack Stress")]
async fn main() {
    let gravity = Vec2::new(0.0, -9.81);
    let mut world = World::new(gravity, Integrator::SemiImplicitEuler);
    world.solver.params.restitution = 0.1;
    world.solver.params.friction = 0.8;
    world.solver.iterations = 20;

    let scale = 60.0;
    let fixed_dt = 1.0 / 240.0;
    let mut accumulator = 0.0f32;

    // Static ground
    let ground = RigidBody::box_xy(Vec2::new(0.0, -6.0), 0.0, 0.0, 30.0, 1.0);
    world.add(Box::new(ground));

    // A tall stack / pyramid hybrid (stress test)
    let box_w = 1.0;
    let box_h = 0.5;
    let mass = 1.0;
    let rows = 14;
    let gap = 0.02;
    let start_y = -6.0 + 1.0 * 0.5 + box_h * 0.5 + 0.02;

    for row in 0..rows {
        let count = rows - row;
        let y = start_y + (box_h + gap) * row as f32;

        let row_width = count as f32 * box_w + (count as f32 - 1.0) * gap;
        let x_start = -0.5 * row_width + 0.5 * box_w;
        for i in 0..count {
            let x = x_start + i as f32 * (box_w + gap);
            let mut rb = RigidBody::box_xy(Vec2::new(x, y), 0.0, mass, box_w, box_h);
            // Tiny perturbation to prevent perfectly symmetric stacking artifacts.
            rb.angle = (row as f32 * 0.001) - 0.007;
            world.add(Box::new(rb));
        }
    }

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
