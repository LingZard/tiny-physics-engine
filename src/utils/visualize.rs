use macroquad::prelude as mq;
use std::any::Any;

use crate::core::World;
use crate::core::entity::PhysicalEntity;
use crate::core::{particle::Particle, rigid_body::RigidBody, shape::Collider2D};
use crate::forces::{
    drag::LinearDrag,
    spring::{Spring, SpringEnd},
};
use crate::math::vec::Vec2;

pub trait Drawable {
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

fn draw_collider_at(pos: &Vec2, angle: f32, collider: &Collider2D, scale: f32) {
    match collider {
        Collider2D::Circle { radius } => {
            let (sx, sy) = to_screen(pos, scale);
            let rpx = radius * scale;
            mq::draw_circle_lines(sx, sy, rpx, 2.0, mq::YELLOW);
            let dir = Vec2::new(angle.cos(), angle.sin());
            let tip = Vec2::new(pos.x + dir.x * radius, pos.y + dir.y * radius);
            let (tx, ty) = to_screen(&tip, scale);
            mq::draw_line(sx, sy, tx, ty, 2.0, mq::ORANGE);
        }
        Collider2D::Box { half_extents } => {
            let (cx, cy) = to_screen(pos, scale);
            let hw = half_extents.x * scale;
            let hh = half_extents.y * scale;
            let corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)];
            let c = angle.cos();
            let s = angle.sin();
            let mut pts = Vec::with_capacity(4);
            for (x, y) in corners {
                let rx = x * c - y * s;
                let ry = x * s + y * c;
                pts.push((cx + rx, cy - ry));
            }
            for i in 0..4 {
                let (x0, y0) = pts[i];
                let (x1, y1) = pts[(i + 1) % 4];
                mq::draw_line(x0, y0, x1, y1, 2.0, mq::YELLOW);
            }
        }
    }
}

fn draw_entities(world: &World, scale: f32) {
    for e in world.entities.iter() {
        let any: &dyn Any = e.as_ref();
        if let Some(rb) = any.downcast_ref::<RigidBody>() {
            rb.draw(world, scale);
            continue;
        }
        if let Some(p) = any.downcast_ref::<Particle>() {
            p.draw(world, scale);
            continue;
        }
        let (sx, sy) = to_screen(e.pos(), scale);
        mq::draw_circle(sx, sy, 5.0, mq::WHITE);
    }
}

pub fn draw_world(world: &World, scale: f32) {
    mq::clear_background(mq::Color::from_rgba(18, 18, 24, 255));
    draw_axes_and_ground();
    draw_forces(world, scale);
    draw_entities(world, scale);
    draw_contacts(world, scale);
    draw_hud(world);
}

fn draw_contacts(world: &World, scale: f32) {
    for manifold in &world.manifolds {
        let normal = &manifold.normal;
        for cp in &manifold.points {
            let (sx, sy) = to_screen(&cp.point, scale);
            // Draw contact point
            mq::draw_circle(sx, sy, 5.0, mq::RED);
            // Draw normal
            let normal_len = 0.3; // world units
            let tip = &cp.point + &(normal * normal_len);
            let (tx, ty) = to_screen(&tip, scale);
            mq::draw_line(sx, sy, tx, ty, 2.0, mq::GREEN);
            // Draw penetration text
            let text = format!("{:.3}", cp.penetration);
            mq::draw_text(&text, sx + 8.0, sy - 8.0, 16.0, mq::WHITE);
        }
    }
}

impl Drawable for Spring {
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

impl Drawable for LinearDrag {
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
    let contact_count: usize = world.manifolds.iter().map(|m| m.points.len()).sum();
    let text = format!(
        "K={:.2}  U={:.2}  E={:.2}  P=({:.2},{:.2})  N={}  C={}",
        kinetic,
        potential,
        kinetic + potential,
        px,
        py,
        world.entities.len(),
        contact_count
    );
    mq::draw_text(&text, 16.0, 24.0, 22.0, mq::WHITE);
}

impl Drawable for RigidBody {
    fn draw(&self, _world: &World, scale: f32) {
        if let Some(col) = &self.collider {
            draw_collider_at(&self.pos, self.angle, col, scale);
        } else {
            let (sx, sy) = to_screen(&self.pos, scale);
            mq::draw_circle(sx, sy, 6.0, mq::YELLOW);
        }
    }
}

impl Drawable for Particle {
    fn draw(&self, _world: &World, scale: f32) {
        let (sx, sy) = to_screen(self.pos(), scale);
        mq::draw_circle(sx, sy, 6.0, mq::YELLOW);
    }
}
