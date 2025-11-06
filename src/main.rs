mod core;
mod forces;
mod math;
mod visualize;

use math::vec::Vec2;

use macroquad::prelude as mq;

use core::{Integrator, World, particle::Particle};
use forces::spring::{AnchoredSpring, Spring};
use visualize::draw_world;

fn resolve_ground_collisions(world: &mut World, scale: f32, restitution: f32) {
    let ground_y = -(mq::screen_height() * 0.5) / scale;
    let radius_world = 6.0 / scale;
    for e in world.entities.iter_mut() {
        let bottom = e.pos().y - radius_world;
        if bottom < ground_y {
            {
                let p = e.pos_mut();
                p.y = ground_y + radius_world;
            }
            {
                let v = e.vel_mut();
                v.y = -v.y * restitution;
                v.x *= 0.98;
                if v.y.abs() < 0.1 {
                    v.y = 0.0;
                }
                if v.x.abs() < 0.05 {
                    v.x = 0.0;
                }
            }
        }
    }
}

fn step_world_fixed(
    world: &mut World,
    mut frame_dt: f32,
    fixed_dt: f32,
    scale: f32,
    restitution: f32,
    accumulator: &mut f32,
) {
    if frame_dt > 1.0 / 30.0 {
        frame_dt = 1.0 / 30.0;
    }
    *accumulator += frame_dt;
    while *accumulator >= fixed_dt {
        world.step(fixed_dt);
        resolve_ground_collisions(world, scale, restitution);
        *accumulator -= fixed_dt;
    }
}

#[macroquad::main("Tiny Physics — Particles")]
async fn main() {
    let gravity = Vec2::new(0.0, -9.81);
    let mut world = World::new(gravity, Integrator::SemiImplicitEuler);

    let seeds = [
        (Vec2::new(-2.0, 6.0), Vec2::new(2.0, 0.0)),
        (Vec2::new(-1.0, 5.5), Vec2::new(1.5, 0.0)),
        (Vec2::new(0.0, 5.0), Vec2::new(0.5, 0.0)),
        (Vec2::new(1.0, 5.8), Vec2::new(-1.0, 0.0)),
        (Vec2::new(2.0, 6.2), Vec2::new(-1.8, 0.0)),
    ];
    for (p0, v0) in seeds {
        world.add(Box::new(Particle::new(p0, v0, 1.0)));
    }

    let scale = 60.0;
    let restitution = 0.7;
    let fixed_dt = 1.0 / 120.0;
    let mut accumulator = 0.0f32;

    if world.entities.len() >= 2 {
        let p0 = world.entities[0].pos();
        let p1 = world.entities[1].pos();
        let rest = (p1 - p0).length();
        world.add_force(Box::new(Spring {
            i: 0,
            j: 1,
            k: 15.0,
            c: 1.5,
            rest,
        }));
    }
    if world.entities.len() >= 3 {
        let anchor = Vec2::new(-3.0, 7.5);
        let p2 = world.entities[2].pos();
        let rest = (p2 - &anchor).length();
        world.add_force(Box::new(AnchoredSpring {
            i: 2,
            anchor,
            k: 20.0,
            c: 2.5,
            rest,
        }));
    }

    loop {
        let frame_dt = mq::get_frame_time();
        step_world_fixed(
            &mut world,
            frame_dt,
            fixed_dt,
            scale,
            restitution,
            &mut accumulator,
        );

        draw_world(&world, scale);

        mq::next_frame().await;
    }
}
