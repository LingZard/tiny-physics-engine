use macroquad::prelude as mq;

use tiny_physics_engine::core::{Integrator, World, rigid_body::RigidBody};
use tiny_physics_engine::math::vec::Vec2;

#[cfg(not(feature = "visualize"))]
compile_error!(
    "Enable this example with: cargo run --example box_stack_contacts --features visualize"
);
#[cfg(feature = "visualize")]
use tiny_physics_engine::utils::visualize::draw_world;

#[macroquad::main("Tiny Physics — Box Stack")]
async fn main() {
    let gravity = Vec2::new(0.0, -9.81);
    let mut world = World::new(gravity, Integrator::SemiImplicitEuler);
    world.restitution = 0.2;
    world.friction = 0.5;
    world.solver.iterations = 20;

    let scale = 60.0;
    let fixed_dt = 1.0 / 240.0;
    let mut accumulator = 0.0f32;

    // Static ground
    let ground_height = 0.5;
    let ground_width = 20.0;
    let ground_y = -4.0 + ground_height * 0.5;
    let ground = RigidBody::box_xy(
        Vec2::new(0.0, ground_y),
        0.0,
        0.0, // mass = 0 -> static
        ground_width,
        ground_height,
    );
    world.add(Box::new(ground));

    // Stack of boxes
    let box_w = 1.0;
    let box_h = 0.5;
    let box_mass = 1.0;
    let stack_count = 5;
    let start_y = ground_y + ground_height * 0.5 + box_h * 0.5 + 0.01;

    for i in 0..stack_count {
        let y = start_y + (box_h + 0.02) * i as f32;
        // Slight offset for stability test
        let x = 0.02 * (i % 2) as f32;
        let rb = RigidBody::box_xy(Vec2::new(x, y), 0.0, box_mass, box_w, box_h);
        world.add(Box::new(rb));
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
