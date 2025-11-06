use crate::core::World;
use crate::forces::{
    drag::LinearDrag,
    spring::{AnchoredSpring, Spring},
};
use crate::math::vec::Vec2;
use macroquad::prelude as mq;
use std::any::Any;

// AI-generated code, for visualization only

pub trait VisualForce {
    fn draw(&self, _world: &World, _scale: f32) {}
}

fn to_screen(p: &Vec2, scale: f32) -> (f32, f32) {
    let cx = mq::screen_width() * 0.5;
    let cy = mq::screen_height() * 0.5;
    (cx + p.x * scale, cy - p.y * scale)
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

fn draw_forces(world: &World, scale: f32) {
    for g in world.forces.iter() {
        let any: &dyn Any = g.as_ref();
        if let Some(s) = any.downcast_ref::<Spring>() {
            s.draw(world, scale);
            continue;
        }
        if let Some(s) = any.downcast_ref::<AnchoredSpring>() {
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
}

impl VisualForce for Spring {
    fn draw(&self, world: &World, scale: f32) {
        let i = self.i;
        let j = self.j;
        if i < world.entities.len() && j < world.entities.len() {
            let p0 = world.entities[i].pos();
            let p1 = world.entities[j].pos();
            let (x0, y0) = to_screen(p0, scale);
            let (x1, y1) = to_screen(p1, scale);
            mq::draw_line(x0, y0, x1, y1, 2.0, mq::ORANGE);
        }
    }
}

impl VisualForce for AnchoredSpring {
    fn draw(&self, world: &World, scale: f32) {
        let i = self.i;
        if i < world.entities.len() {
            let pi = world.entities[i].pos();
            let (ax, ay) = to_screen(&self.anchor, scale);
            let (ix, iy) = to_screen(pi, scale);
            mq::draw_circle(ax, ay, 4.0, mq::RED);
            mq::draw_line(ax, ay, ix, iy, 2.0, mq::RED);
        }
    }
}

impl VisualForce for LinearDrag {
    fn draw(&self, _world: &World, _scale: f32) {
        // no-op visualization for drag
    }
}
