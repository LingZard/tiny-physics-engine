use super::ForceGen;
use crate::core::World;
use crate::math::vec::Vec2;

pub enum SpringEnd {
    Entity(usize),
    Anchor(Vec2),
}

pub struct Spring {
    pub a: SpringEnd,
    pub b: SpringEnd,
    pub k: f32,
    pub c: f32,
    pub rest: f32,
}

impl Spring {
    pub fn between(i: usize, j: usize, k: f32, c: f32, rest: f32) -> Self {
        Self {
            a: SpringEnd::Entity(i),
            b: SpringEnd::Entity(j),
            k,
            c,
            rest,
        }
    }

    pub fn to_anchor(i: usize, anchor: Vec2, k: f32, c: f32, rest: f32) -> Self {
        Self {
            a: SpringEnd::Entity(i),
            b: SpringEnd::Anchor(anchor),
            k,
            c,
            rest,
        }
    }
}

impl ForceGen for Spring {
    fn apply(&self, world: &mut World) {
        let (pa, va, inv_ma) = match &self.a {
            SpringEnd::Entity(i) => {
                let e = world
                    .entities
                    .get(*i)
                    .map(|e| (*e.pos(), *e.vel(), e.inv_mass()));
                match e {
                    Some(v) => v,
                    None => return,
                }
            }
            SpringEnd::Anchor(p) => (*p, Vec2::zero(), 0.0),
        };

        let (pb, vb, inv_mb) = match &self.b {
            SpringEnd::Entity(j) => {
                let e = world
                    .entities
                    .get(*j)
                    .map(|e| (*e.pos(), *e.vel(), e.inv_mass()));
                match e {
                    Some(v) => v,
                    None => return,
                }
            }
            SpringEnd::Anchor(p) => (*p, Vec2::zero(), 0.0),
        };

        let displacement = pa - pb;
        let distance = displacement.length();
        if distance < 1e-6 {
            return;
        }

        let direction = displacement / distance;
        let extension = distance - self.rest;
        let f_spring = direction * (-self.k * extension);
        let v_rel = va - vb;
        let axial = v_rel.dot(direction);
        let f_damp = direction * (-self.c * axial);
        let f_a = f_spring + f_damp;

        if let SpringEnd::Entity(i) = self.a {
            if inv_ma > 0.0 {
                let e = &mut world.entities[i];
                *e.force_mut() = *e.force() + f_a;
            }
        }
        if let SpringEnd::Entity(j) = self.b {
            if inv_mb > 0.0 {
                let e = &mut world.entities[j];
                *e.force_mut() = *e.force() - f_a;
            }
        }
    }
}
