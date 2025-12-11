use super::body::PhysicalEntity;
use super::collision::{Manifold, broad_phase, narrow_phase};
use super::integrator::Integrator;
use super::solver::ConstraintSolver;
use crate::forces::ForceGen;
use crate::math::vec::Vec2;

pub struct World {
    pub gravity: Vec2,
    pub integrator: Integrator,
    pub entities: Vec<Box<dyn PhysicalEntity>>,
    pub forces: Vec<Box<dyn ForceGen>>,
    pub solver: ConstraintSolver,
    pub manifolds: Vec<Manifold>,
}

impl World {
    pub fn new(gravity: Vec2, integrator: Integrator) -> Self {
        Self {
            gravity,
            integrator,
            entities: Vec::new(),
            forces: Vec::new(),
            solver: ConstraintSolver::new(10),
            manifolds: Vec::new(),
        }
    }

    pub fn add(&mut self, entity: Box<dyn PhysicalEntity>) {
        self.entities.push(entity);
    }

    pub fn add_force(&mut self, force: Box<dyn ForceGen>) {
        self.forces.push(force);
    }

    pub fn step(&mut self, dt: f32) {
        if dt <= 0.0 {
            return;
        }

        for e in &mut self.entities {
            e.clear_forces();
            e.clear_torque();
        }

        for e in &mut self.entities {
            if e.inv_mass() > 0.0 {
                let mass = 1.0 / e.inv_mass();
                *e.force_mut() = *e.force() + self.gravity * mass;
            }
        }

        let forces = core::mem::take(&mut self.forces);
        for f in &forces {
            f.apply(self);
        }
        self.forces = forces;

        for e in &mut self.entities {
            if e.inv_mass() > 0.0 {
                *e.vel_mut() = *e.vel() + *e.force() * e.inv_mass() * dt;
            }
            if e.inv_inertia() > 0.0 {
                *e.omega_mut() = e.omega() + e.torque() * e.inv_inertia() * dt;
            }
        }

        let pairs = broad_phase::detect_sap(&self.entities);
        self.manifolds = narrow_phase::detect(&self.entities, &pairs);

        self.solver
            .build_constraints(&self.manifolds, &self.entities, dt);
        self.solver.solve(&mut self.entities);

        for e in &mut self.entities {
            *e.pos_mut() = *e.pos() + *e.vel() * dt;
            *e.angle_mut() = e.angle() + e.omega() * dt;
        }
    }
}
