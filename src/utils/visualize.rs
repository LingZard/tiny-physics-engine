use macroquad::prelude as mq;
use std::any::Any;

use crate::core::World;
use crate::forces::{
    drag::LinearDrag,
    spring::{Spring, SpringEnd},
};
use crate::math::vec::Vec2;

pub trait VisualForce {
    fn draw(&self, _world: &World, _scale: f32) {}
}

fn to_screen(p: &Vec2, scale: f32) -> (f32, f32) {
    let cx = mq::screen_width() * 0.5;
    let cy = mq::screen_height() * 0.5;
    (cx + p.x * scale, cy - p.y * scale)
}

pub fn draw_axes_and_ground() {
    let cx = mq::screen_width() * 0.5;
    let cy = mq::screen_height() * 0.5;
    mq::draw_line(0.0, cy, mq::screen_width(), cy, 1.0, mq::GRAY);
    mq::draw_line(cx, 0.0, cx, mq::screen_height(), 1.0, mq::GRAY);
    let bottom_y = mq::screen_height() - 1.0;
    mq::draw_line(0.0, bottom_y, mq::screen_width(), bottom_y, 2.0, mq::GREEN);
}

pub fn draw_particles(world: &World, scale: f32) {
    for e in world.entities.iter() {
        let pos = e.pos();
        let (sx, sy) = to_screen(pos, scale);
        mq::draw_circle(sx, sy, 6.0, mq::YELLOW);
    }
}

fn draw_forces(world: &World, scale: f32) {
    for g in world.forces.iter() {
        let any: &dyn Any = g.as_ref();
        if let Some(s) = any.downcast_ref::<Spring>() {
            s.draw(world, scale);
            continue;
        }
        if let Some(s) = any.downcast_ref::<LinearDrag>() {
            s.draw(world, scale);
            continue;
        }
    }
}

pub fn draw_world(world: &World, scale: f32) {
    mq::clear_background(mq::Color::from_rgba(18, 18, 24, 255));
    draw_axes_and_ground();
    draw_forces(world, scale);
    draw_particles(world, scale);
    draw_hud(world);
}

impl VisualForce for Spring {
    fn draw(&self, world: &World, scale: f32) {
        let p_of = |end: &SpringEnd| -> Option<Vec2> {
            match end {
                SpringEnd::Entity(i) => world
                    .entities
                    .get(*i)
                    .map(|e| Vec2::new(e.pos().x, e.pos().y)),
                SpringEnd::Anchor(p) => Some(Vec2::new(p.x, p.y)),
            }
        };
        if let (Some(pa), Some(pb)) = (p_of(&self.a), p_of(&self.b)) {
            let (x0, y0) = to_screen(&pa, scale);
            let (x1, y1) = to_screen(&pb, scale);
            mq::draw_line(x0, y0, x1, y1, 2.0, mq::ORANGE);
            if let SpringEnd::Anchor(ref p) = self.a {
                let (ax, ay) = to_screen(p, scale);
                mq::draw_circle(ax, ay, 4.0, mq::RED);
            }
            if let SpringEnd::Anchor(ref p) = self.b {
                let (ax, ay) = to_screen(p, scale);
                mq::draw_circle(ax, ay, 4.0, mq::RED);
            }
        }
    }
}

impl VisualForce for LinearDrag {
    fn draw(&self, _world: &World, _scale: f32) {}
}

fn draw_hud(world: &World) {
    let mut kinetic = 0.0f32;
    let mut px = 0.0f32;
    let mut py = 0.0f32;
    for e in world.entities.iter() {
        let inv_m = e.inv_mass();
        if inv_m > 0.0 {
            let m = 1.0 / inv_m;
            let v = e.vel();
            kinetic += 0.5 * m * (v.x * v.x + v.y * v.y);
            px += m * v.x;
            py += m * v.y;
        }
    }
    let mut potential = 0.0f32;
    for g in world.forces.iter() {
        let any: &dyn Any = g.as_ref();
        if let Some(s) = any.downcast_ref::<Spring>() {
            let p_of = |end: &SpringEnd| -> Option<Vec2> {
                match end {
                    SpringEnd::Entity(i) => world
                        .entities
                        .get(*i)
                        .map(|e| Vec2::new(e.pos().x, e.pos().y)),
                    SpringEnd::Anchor(p) => Some(Vec2::new(p.x, p.y)),
                }
            };
            if let (Some(pa), Some(pb)) = (p_of(&s.a), p_of(&s.b)) {
                let d = pa - &pb;
                let dist = d.length();
                let x = dist - s.rest;
                potential += 0.5 * s.k * x * x;
            }
        }
    }
    let text = format!(
        "K={:.2}  U={:.2}  E={:.2}  P=({:.2},{:.2})  N={}",
        kinetic,
        potential,
        kinetic + potential,
        px,
        py,
        world.entities.len()
    );
    mq::draw_text(&text, 16.0, 24.0, 22.0, mq::WHITE);
}
