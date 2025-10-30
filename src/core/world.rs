use super::{
    entity::PhysicalEntity,
    integrator::{Integrator, integrate},
};
use crate::math::vec::Vec2;

pub struct World {
    pub gravity: Vec2,
    pub integrator: Integrator,
    pub entities: Vec<Box<dyn PhysicalEntity>>,
}

impl World {
    pub fn new(gravity: Vec2, integrator: Integrator) -> Self {
        Self {
            gravity,
            integrator,
            entities: Vec::new(),
        }
    }

    pub fn add(&mut self, entity: Box<dyn PhysicalEntity>) {
        self.entities.push(entity);
    }

    pub fn step(&mut self, dt: f32) {
        for e in self.entities.iter_mut() {
            let inv_m = e.inv_mass();
            if inv_m > 0.0 {
                let mass = 1.0 / inv_m;
                let fg = &self.gravity * mass;
                let f = e.force() + &fg;
                *e.force_mut() = f;
            }
        }
        for e in self.entities.iter_mut() {
            integrate(&mut (**e), dt, self.integrator);
            e.clear_forces();
        }
    }
}
