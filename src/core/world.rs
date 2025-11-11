use super::{
    entity::PhysicalEntity,
    integrator::{Integrator, integrate},
};
use crate::forces::ForceGen;
use crate::math::vec::Vec2;

pub struct World {
    pub gravity: Vec2,
    pub integrator: Integrator,
    pub entities: Vec<Box<dyn PhysicalEntity>>,
    pub forces: Vec<Box<dyn ForceGen>>,
}

impl World {
    pub fn new(gravity: Vec2, integrator: Integrator) -> Self {
        Self {
            gravity,
            integrator,
            entities: Vec::new(),
            forces: Vec::new(),
        }
    }

    pub fn add(&mut self, entity: Box<dyn PhysicalEntity>) {
        self.entities.push(entity);
    }

    pub fn add_force(&mut self, force: Box<dyn crate::forces::ForceGen>) {
        self.forces.push(force);
    }

    pub fn step(&mut self, dt: f32) {
        // 1) clear
        for e in self.entities.iter_mut() {
            e.clear_forces();
            e.clear_torque();
        }

        // 2) gravity
        for e in self.entities.iter_mut() {
            let inv_m = e.inv_mass();
            if inv_m > 0.0 {
                let mass = 1.0 / inv_m;
                let fg = &self.gravity * mass;
                let f = e.force() + &fg;
                *e.force_mut() = f;
            }
        }

        // 3) external forces (avoid aliasing borrows)
        let forces = core::mem::take(&mut self.forces);
        for g in forces.iter() {
            g.apply(self);
        }
        self.forces = forces;

        // 4) integrate
        for e in self.entities.iter_mut() {
            integrate(&mut (**e), dt, self.integrator);
        }
    }
}
