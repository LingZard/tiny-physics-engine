mod core;
mod forces;
mod math;

use math::vec::Vec2;

use macroquad::prelude as mq;

use core::{Integrator, World, particle::Particle};

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

    loop {
        let dt = mq::get_frame_time();

        world.step(dt);
        resolve_ground_collisions(&mut world, scale, restitution);

        mq::clear_background(mq::Color::from_rgba(18, 18, 24, 255));

        let cx = mq::screen_width() * 0.5;
        let cy = mq::screen_height() * 0.5;
        mq::draw_line(0.0, cy, mq::screen_width(), cy, 1.0, mq::GRAY);
        mq::draw_line(cx, 0.0, cx, mq::screen_height(), 1.0, mq::GRAY);
        let bottom_y = mq::screen_height() - 1.0;
        mq::draw_line(0.0, bottom_y, mq::screen_width(), bottom_y, 2.0, mq::GREEN);

        for e in world.entities.iter() {
            let pos = e.pos();
            let (sx, sy) = to_screen(pos, scale);
            mq::draw_circle(sx, sy, 6.0, mq::YELLOW);
        }

        let e = &world.entities[0];
        let pos = e.pos();
        let vel = e.vel();
        mq::draw_text(
            &format!(
                "pos=({:.2},{:.2}) vel=({:.2},{:.2}) dt={:.3}",
                pos.x, pos.y, vel.x, vel.y, dt
            ),
            16.0,
            28.0,
            22.0,
            mq::WHITE,
        );

        mq::next_frame().await;
    }
}
