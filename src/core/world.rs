use super::body::PhysicalEntity;
use super::collision::{Manifold, broad_phase, narrow_phase};
use super::integrator::{Integrator, integrate_velocity};
use super::params::SimParams;
use super::solver::ConstraintSolver;
use crate::forces::ForceGen;
use crate::math::vec::Vec2;

pub struct World {
    pub gravity: Vec2,
    pub integrator: Integrator,
    pub params: SimParams,
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
            params: SimParams::default(),
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

    /// TGS-style simulation step:
    /// 1) clear accumulators
    /// 2) apply gravity + external forces
    /// 3) integrate velocity
    /// 4) collision detect (broad + narrow)
    /// 5) solve contacts (TGS: solver predicts per-body motion internally)
    /// 6) integrate position
    pub fn step(&mut self, dt: f32) {
        if dt <= 0.0 {
            return;
        }

        // (1) Clear accumulators.
        for e in &mut self.entities {
            e.clear_forces();
            e.clear_torque();
        }

        // (2) Apply gravity as force: F = m * g.
        for e in &mut self.entities {
            if e.inv_mass() > 0.0 {
                let mass = 1.0 / e.inv_mass();
                *e.force_mut() = *e.force() + self.gravity * mass;
            }
        }

        // (2b) Apply user force generators (springs, drag, ...).
        let forces = core::mem::take(&mut self.forces);
        for f in &forces {
            f.apply(self);
        }
        self.forces = forces;

        // (3) Integrate velocities from accumulated force/torque.
        for e in &mut self.entities {
            integrate_velocity(&mut **e, dt, self.integrator);
        }

        // (4) Detect collisions at current configuration.
        let pairs = broad_phase::detect_sap(&self.entities, self.params);
        self.manifolds = narrow_phase::detect(&self.entities, &pairs, self.params);

        // (5) Build constraints and solve (TGS-style: uses delta tracking).
        self.solver
            .build_constraints(&self.manifolds, &self.entities, dt);
        self.solver.solve(&mut self.entities);

        // (6) Integrate positions.
        for e in &mut self.entities {
            let dp = *e.vel() * dt;
            let da = e.omega() * dt;
            *e.pos_mut() = *e.pos() + dp;
            *e.angle_mut() = e.angle() + da;
        }
    }
}
