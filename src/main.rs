mod core;
mod forces;
mod math;

use math::vec::Vec2;

use macroquad::prelude as mq;

use core::{Integrator, World, particle::Particle};
use forces::{
    drag::LinearDrag,
    spring::{AnchoredSpring, Spring},
};

fn to_screen(p: &Vec2, scale: f32) -> (f32, f32) {
    let cx = mq::screen_width() * 0.5;
    let cy = mq::screen_height() * 0.5;
    (cx + p.x * scale, cy - p.y * scale)
}

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

fn draw_axes_and_ground() {
    let cx = mq::screen_width() * 0.5;
    let cy = mq::screen_height() * 0.5;
    mq::draw_line(0.0, cy, mq::screen_width(), cy, 1.0, mq::GRAY);
    mq::draw_line(cx, 0.0, cx, mq::screen_height(), 1.0, mq::GRAY);
    let bottom_y = mq::screen_height() - 1.0;
    mq::draw_line(0.0, bottom_y, mq::screen_width(), bottom_y, 2.0, mq::GREEN);
}

fn draw_particles(world: &World, scale: f32) {
    for e in world.entities.iter() {
        let pos = e.pos();
        let (sx, sy) = to_screen(pos, scale);
        mq::draw_circle(sx, sy, 6.0, mq::YELLOW);
    }
}

fn draw_springs(world: &World, scale: f32, anchor: Option<&Vec2>) {
    if world.entities.len() >= 2 {
        let p0 = world.entities[0].pos();
        let p1 = world.entities[1].pos();
        let (x0, y0) = to_screen(p0, scale);
        let (x1, y1) = to_screen(p1, scale);
        mq::draw_line(x0, y0, x1, y1, 2.0, mq::ORANGE);
    }
    if let Some(a) = anchor {
        if world.entities.len() >= 3 {
            let p2 = world.entities[2].pos();
            let (ax, ay) = to_screen(a, scale);
            let (x2, y2) = to_screen(p2, scale);
            mq::draw_circle(ax, ay, 4.0, mq::RED);
            mq::draw_line(ax, ay, x2, y2, 2.0, mq::RED);
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

    // world.add_force(Box::new(LinearDrag { k: 0.8 }));
    let mut anchor_draw: Option<Vec2> = None;
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
        anchor_draw = Some(Vec2::new(-3.0, 7.5));
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

        mq::clear_background(mq::Color::from_rgba(18, 18, 24, 255));
        draw_axes_and_ground();
        draw_springs(&world, scale, anchor_draw.as_ref());
        draw_particles(&world, scale);

        mq::next_frame().await;
    }
}
