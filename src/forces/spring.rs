use crate::core::World;
use crate::math::vec::Vec2;

use super::ForceGen;

pub struct Spring {
    pub i: usize,
    pub j: usize,
    pub k: f32,
    pub c: f32,
    pub rest: f32,
}

impl ForceGen for Spring {
    fn apply(&self, world: &mut World) {
        if self.i == self.j {
            return;
        }
        let (i, j) = if self.i < self.j {
            (self.i, self.j)
        } else {
            (self.j, self.i)
        };
        if j >= world.entities.len() {
            return;
        }
        let (left, right) = world.entities.split_at_mut(j);
        let a = &mut left[i];
        let b = &mut right[0];

        let pa = a.pos();
        let pb = b.pos();
        let dir = pb - pa; // from a to b
        let len = dir.length();
        if len == 0.0 {
            return;
        }
        let n = dir / len;

        // Hooke
        let x = len - self.rest;
        let fs = -self.k * x;

        // Damping along spring direction
        let vr = (b.vel() - a.vel()).dot(&n);
        let fd = -self.c * vr;

        let f = n * (fs + fd); // apply to a, opposite to b
        if a.inv_mass() > 0.0 {
            let fa = a.force() + &f;
            *a.force_mut() = fa;
        }
        if b.inv_mass() > 0.0 {
            let fb = b.force() - &f;
            *b.force_mut() = fb;
        }
    }
}

pub struct AnchoredSpring {
    pub i: usize,
    pub anchor: Vec2,
    pub k: f32,
    pub c: f32,
    pub rest: f32,
}

impl ForceGen for AnchoredSpring {
    fn apply(&self, world: &mut World) {
        if self.i >= world.entities.len() {
            return;
        }
        let e = &mut world.entities[self.i];
        let p = e.pos();
        let dir = p - &self.anchor; // from anchor to particle
        let len = dir.length();
        if len == 0.0 {
            return;
        }
        let n = dir / len;
        let x = len - self.rest;
        let fs = -self.k * x;
        let vr = e.vel().dot(&n);
        let fd = -self.c * vr;
        let f = n * (fs + fd);
        if e.inv_mass() > 0.0 {
            let fe = e.force() + &f;
            *e.force_mut() = fe;
        }
    }
}
