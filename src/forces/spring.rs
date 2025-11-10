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

        let (i, j) = (self.i.min(self.j), self.i.max(self.j));
        if j >= world.entities.len() {
            return;
        }

        let (left, right) = world.entities.split_at_mut(j);
        let a = &mut left[i];
        let b = &mut right[0];

        let displacement = a.pos() - b.pos();
        let distance = displacement.length();
        if distance < 1e-6 {
            return;
        }
        let direction = displacement / distance;

        let extension = distance - self.rest;
        let spring_force_magnitude = -self.k * extension;

        let v_rel = a.vel() - b.vel();
        let f_damp = -(&v_rel * self.c);

        let f_spring = &direction * spring_force_magnitude;
        let force_on_a = &f_spring + &f_damp;

        if a.inv_mass() > 0.0 {
            let updated = a.force() + &force_on_a;
            *a.force_mut() = updated;
        }
        if b.inv_mass() > 0.0 {
            let updated = b.force() - &force_on_a;
            *b.force_mut() = updated;
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

        let entity = &mut world.entities[self.i];
        let displacement = entity.pos() - &self.anchor;
        let distance = displacement.length();
        if distance < 1e-6 {
            return;
        }
        let direction = displacement / distance;

        let extension = distance - self.rest;
        let spring_force_magnitude = -self.k * extension;
        let relative_speed_along = entity.vel().dot(&direction);
        let damping_magnitude = -self.c * relative_speed_along;
        let force = direction * (spring_force_magnitude + damping_magnitude);

        if entity.inv_mass() > 0.0 {
            let updated = entity.force() + &force;
            *entity.force_mut() = updated;
        }
    }
}
