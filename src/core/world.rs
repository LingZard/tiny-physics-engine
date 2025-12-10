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
    pub restitution: f32,
    pub friction: f32,
    /// Debug: last frame's collision manifolds
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
            restitution: 0.3,
            friction: 0.5,
            manifolds: Vec::new(),
        }
    }

    pub fn add(&mut self, entity: Box<dyn PhysicalEntity>) {
        self.entities.push(entity);
    }

    pub fn add_force(&mut self, force: Box<dyn crate::forces::ForceGen>) {
        self.forces.push(force);
    }

    pub fn step(&mut self, dt: f32) {
        if dt <= 0.0 {
            return;
        }

        for e in self.entities.iter_mut() {
            e.clear_forces();
            e.clear_torque();
        }

        for e in self.entities.iter_mut() {
            let inv_m = e.inv_mass();
            if inv_m > 0.0 {
                let mass = 1.0 / inv_m;
                let fg = &self.gravity * mass;
                let f = e.force() + &fg;
                *e.force_mut() = f;
            }
        }

        let forces = core::mem::take(&mut self.forces);
        for g in forces.iter() {
            g.apply(self);
        }
        self.forces = forces;

        for e in self.entities.iter_mut() {
            if e.inv_mass() > 0.0 {
                let acc = e.force() * e.inv_mass();
                *e.vel_mut() = e.vel() + &(acc * dt);
            }
            if e.inv_inertia() > 0.0 {
                let alpha = e.torque() * e.inv_inertia();
                *e.omega_mut() = e.omega() + alpha * dt;
            }
        }

        let pairs = broad_phase::detect_sap(&self.entities);
        let manifolds = narrow_phase::detect(&self.entities, &pairs);

        self.manifolds = manifolds;

        self.solver.build_constraints(
            &self.manifolds,
            &self.entities,
            self.restitution,
            self.friction,
            dt,
        );

        self.solver.solve(&mut self.entities);

        for e in self.entities.iter_mut() {
            let new_pos = e.pos() + &(e.vel() * dt);
            *e.pos_mut() = new_pos;
            *e.angle_mut() = e.angle() + e.omega() * dt;
        }
    }
}
