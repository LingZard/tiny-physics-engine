use macroquad::prelude as mq;

use tiny_physics_engine::core::entity::PhysicalEntity;
use tiny_physics_engine::core::{Integrator, World, rigid_body::RigidBody};
use tiny_physics_engine::math::vec::Vec2;

#[cfg(not(feature = "visualize"))]
compile_error!(
    "Enable this example with: cargo run --example simple_rigidbody --features visualize"
);
#[cfg(feature = "visualize")]
use tiny_physics_engine::utils::visualize::draw_world;

fn resolve_ground_collisions_rb(body: &mut dyn PhysicalEntity, scale: f32, restitution: f32) {
    let half_h = (mq::screen_height() * 0.5) / scale;
    let bottom_y = -half_h;
    let radius_world = 8.0 / scale;
    let y_bottom = body.pos().y - radius_world;
    if y_bottom < bottom_y {
        body.pos_mut().y = bottom_y + radius_world;
        let v = body.vel_mut();
        v.y = -v.y * restitution;
        v.x *= 0.98;
        if v.y.abs() < 0.1 {
            v.y = 0.0;
        }
        if v.x.abs() < 0.05 {
            v.x = 0.0;
        }
        // 简单角速度衰减
        let w = body.omega_mut();
        *w *= 0.95;
        if w.abs() < 0.05 {
            *w = 0.0;
        }
    }
}

#[macroquad::main("Tiny Physics — Simple RigidBody")]
async fn main() {
    let gravity = Vec2::new(0.0, -9.81);
    let mut world = World::new(gravity, Integrator::SemiImplicitEuler);
    let scale = 60.0;
    let restitution = 0.6;
    let fixed_dt = 1.0 / 120.0;
    let mut accumulator = 0.0f32;

    let mut rb = RigidBody::box_xy(Vec2::new(0.0, 4.0), 0.3, 1.0, 40.0 / scale, 20.0 / scale);
    rb.omega = 2.5;
    world.add(Box::new(rb));

    loop {
        let mut frame_dt = mq::get_frame_time();
        if frame_dt > 1.0 / 30.0 {
            frame_dt = 1.0 / 30.0;
        }
        accumulator += frame_dt;
        while accumulator >= fixed_dt {
            world.step(fixed_dt);
            if let Some(e) = world.entities.get_mut(0) {
                resolve_ground_collisions_rb(&mut **e, scale, restitution);
            }
            accumulator -= fixed_dt;
        }

        draw_world(&world, scale);

        mq::next_frame().await;
    }
}
