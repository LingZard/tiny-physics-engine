use std::collections::HashMap;

use crate::core::body::PhysicalEntity;
use crate::core::collision::{ContactPoint, Manifold};
use crate::math::vec::Vec2;

#[derive(Debug, Clone)]
pub struct ContactConstraint {
    pub index_a: usize,
    pub index_b: usize,
    pub normal: Vec2,
    pub tangent: Vec2,
    pub anchor_a: Vec2,
    pub anchor_b: Vec2,
    pub base_separation: f32,
    pub normal_mass: f32,
    pub tangent_mass: f32,
    pub jn: f32,
    pub jt: f32,
    /// Initial relative normal velocity (for restitution)
    pub relative_velocity: f32,
}

impl ContactConstraint {
    fn new(
        index_a: usize,
        index_b: usize,
        normal: Vec2,
        cp: &ContactPoint,
        a: &dyn PhysicalEntity,
        b: &dyn PhysicalEntity,
    ) -> Self {
        // Fixed anchors relative to body centers
        let anchor_a = cp.point - *a.pos();
        let anchor_b = cp.point - *b.pos();
        let tangent = normal.perp();

        let eff_mass = |axis: Vec2| {
            let rn_a = anchor_a.cross(axis);
            let rn_b = anchor_b.cross(axis);
            let inv = a.inv_mass()
                + b.inv_mass()
                + rn_a * rn_a * a.inv_inertia()
                + rn_b * rn_b * b.inv_inertia();
            if inv > 1e-8 { 1.0 / inv } else { 0.0 }
        };

        let base_separation = -cp.penetration - (anchor_b - anchor_a).dot(normal);

        // Save relative velocity for restitution (computed once at constraint build time)
        let rel_vel = velocity_at(anchor_b, b) - velocity_at(anchor_a, a);
        let relative_velocity = rel_vel.dot(normal);

        Self {
            index_a,
            index_b,
            normal,
            tangent,
            anchor_a,
            anchor_b,
            base_separation,
            normal_mass: eff_mass(normal),
            tangent_mass: eff_mass(tangent),
            jn: 0.0,
            jt: 0.0,
            relative_velocity,
        }
    }

    /// TGS-style normal constraint solve.
    /// Computes current separation using delta_pos/delta_angle, then applies bias.
    pub fn solve_normal(
        &mut self,
        entities: &mut [Box<dyn PhysicalEntity>],
        inv_dt: f32,
        params: &SolverParams,
        use_bias: bool,
    ) {
        let Some((a, b)) = get_pair_mut(entities, self.index_a, self.index_b) else {
            return;
        };

        // Compute rotated anchors using delta_angle
        let da_a = a.delta_angle();
        let da_b = b.delta_angle();
        let (sin_a, cos_a) = da_a.sin_cos();
        let (sin_b, cos_b) = da_b.sin_cos();

        let rotated_a = Vec2::new(
            self.anchor_a.x * cos_a - self.anchor_a.y * sin_a,
            self.anchor_a.x * sin_a + self.anchor_a.y * cos_a,
        );
        let rotated_b = Vec2::new(
            self.anchor_b.x * cos_b - self.anchor_b.y * sin_b,
            self.anchor_b.x * sin_b + self.anchor_b.y * cos_b,
        );

        // Compute current separation
        let dp = *b.delta_pos() - *a.delta_pos();
        let ds = dp + rotated_b - rotated_a;
        let separation = self.base_separation + ds.dot(self.normal);

        // Compute velocity bias based on separation
        let velocity_bias;

        if separation > 0.0 {
            // Speculative contact: push apart at rate to close gap in one step
            velocity_bias = separation * inv_dt;
        } else if use_bias {
            // Bias for penetration correction (clamped)
            velocity_bias = (params.bias_rate * separation).max(-params.max_bias_velocity);
        } else {
            velocity_bias = 0.0;
        }

        // Relative normal velocity at contact
        let vn = (velocity_at(self.anchor_b, b) - velocity_at(self.anchor_a, a)).dot(self.normal);

        // Incremental normal impulse
        let impulse = -self.normal_mass * (vn + velocity_bias);

        // Clamp the accumulated impulse
        let jn_old = self.jn;
        self.jn = (jn_old + impulse).max(0.0);
        let delta = self.jn - jn_old;

        apply_impulse_pair(a, b, self.anchor_a, self.anchor_b, self.normal, delta);
    }

    pub fn solve_tangent(&mut self, entities: &mut [Box<dyn PhysicalEntity>], friction: f32) {
        let Some((a, b)) = get_pair_mut(entities, self.index_a, self.index_b) else {
            return;
        };

        let vt = (velocity_at(self.anchor_b, b) - velocity_at(self.anchor_a, a)).dot(self.tangent);
        let lambda = -self.tangent_mass * vt;

        let max_jt = friction * self.jn;
        let jt_old = self.jt;
        self.jt = (jt_old + lambda).clamp(-max_jt, max_jt);
        let delta = self.jt - jt_old;

        apply_impulse_pair(a, b, self.anchor_a, self.anchor_b, self.tangent, delta);
    }

    /// Apply restitution impulse (separate pass, like Box2D).
    pub fn apply_restitution(
        &mut self,
        entities: &mut [Box<dyn PhysicalEntity>],
        restitution: f32,
        threshold: f32,
    ) {
        if restitution == 0.0 {
            return;
        }
        if self.relative_velocity > -threshold || self.jn == 0.0 {
            return;
        }

        let Some((a, b)) = get_pair_mut(entities, self.index_a, self.index_b) else {
            return;
        };

        let vn = (velocity_at(self.anchor_b, b) - velocity_at(self.anchor_a, a)).dot(self.normal);
        let impulse = -self.normal_mass * (vn + restitution * self.relative_velocity);

        let jn_old = self.jn;
        self.jn = (jn_old + impulse).max(0.0);
        let delta = self.jn - jn_old;

        apply_impulse_pair(a, b, self.anchor_a, self.anchor_b, self.normal, delta);
    }

    fn apply_warm_start(&self, entities: &mut [Box<dyn PhysicalEntity>]) {
        if self.jn == 0.0 && self.jt == 0.0 {
            return;
        }
        let Some((a, b)) = get_pair_mut(entities, self.index_a, self.index_b) else {
            return;
        };
        apply_impulse_pair(a, b, self.anchor_a, self.anchor_b, self.normal, self.jn);
        apply_impulse_pair(a, b, self.anchor_a, self.anchor_b, self.tangent, self.jt);
    }
}

#[inline]
fn velocity_at(r: Vec2, e: &dyn PhysicalEntity) -> Vec2 {
    *e.vel() + Vec2::new(-e.omega() * r.y, e.omega() * r.x)
}

#[inline]
fn apply_impulse_pair(
    a: &mut dyn PhysicalEntity,
    b: &mut dyn PhysicalEntity,
    r_a: Vec2,
    r_b: Vec2,
    dir: Vec2,
    magnitude: f32,
) {
    let impulse = dir * magnitude;
    *a.vel_mut() = *a.vel() - a.inv_mass() * impulse;
    *a.omega_mut() = a.omega() - a.inv_inertia() * r_a.cross(impulse);
    *b.vel_mut() = *b.vel() + b.inv_mass() * impulse;
    *b.omega_mut() = b.omega() + b.inv_inertia() * r_b.cross(impulse);
}

fn get_pair_mut(
    entities: &mut [Box<dyn PhysicalEntity>],
    i: usize,
    j: usize,
) -> Option<(&mut dyn PhysicalEntity, &mut dyn PhysicalEntity)> {
    if i == j || i >= entities.len() || j >= entities.len() {
        return None;
    }
    let (lo, hi) = if i < j { (i, j) } else { (j, i) };
    let (left, right) = entities.split_at_mut(hi);
    let a = &mut *left[lo];
    let b = &mut *right[0];
    Some(if i < j { (a, b) } else { (b, a) })
}

const CELL_SIZE: f32 = 0.05;

#[derive(Hash, Eq, PartialEq, Clone, Copy)]
struct CacheKey {
    pair: (usize, usize),
    cell: (i32, i32),
}

impl CacheKey {
    fn new(index_a: usize, index_b: usize, anchor_a: Vec2) -> Self {
        Self {
            pair: (index_a, index_b),
            cell: (
                (anchor_a.x / CELL_SIZE).round() as i32,
                (anchor_a.y / CELL_SIZE).round() as i32,
            ),
        }
    }
}

/// TGS-style solver parameters (Box2D v3 approach).
#[derive(Clone)]
pub struct SolverParams {
    /// Bias rate for soft constraint (how fast to correct penetration)
    pub bias_rate: f32,
    /// Maximum bias velocity to prevent explosive corrections
    pub max_bias_velocity: f32,
    /// Restitution threshold (minimum relative velocity for bounce)
    pub restitution_threshold: f32,
    /// Default restitution coefficient
    pub restitution: f32,
    /// Default friction coefficient
    pub friction: f32,
}

impl Default for SolverParams {
    fn default() -> Self {
        // These values are tuned similar to Box2D defaults
        Self {
            bias_rate: 0.2,         // ~30Hz contact hertz equivalent
            max_bias_velocity: 2.0, // Limit correction speed
            restitution_threshold: 1.0,
            restitution: 0.3,
            friction: 0.5,
        }
    }
}

pub struct ConstraintSolver {
    pub constraints: Vec<ContactConstraint>,
    pub iterations: usize,
    pub params: SolverParams,
    cache: HashMap<CacheKey, (f32, f32)>,
    inv_dt: f32,
}

impl ConstraintSolver {
    pub fn new(iterations: usize) -> Self {
        Self {
            constraints: Vec::new(),
            iterations,
            params: SolverParams::default(),
            cache: HashMap::new(),
            inv_dt: 0.0,
        }
    }

    pub fn build_constraints(
        &mut self,
        manifolds: &[Manifold],
        entities: &[Box<dyn PhysicalEntity>],
        dt: f32,
    ) {
        self.inv_dt = if dt > 0.0 { 1.0 / dt } else { 0.0 };

        // Cache old impulses for warm starting
        self.cache.clear();
        for c in &self.constraints {
            if c.jn != 0.0 || c.jt != 0.0 {
                let key = CacheKey::new(c.index_a, c.index_b, c.anchor_a);
                self.cache.insert(key, (c.jn, c.jt));
            }
        }

        self.constraints.clear();

        for manifold in manifolds {
            let (Some(a), Some(b)) = (entities.get(manifold.a), entities.get(manifold.b)) else {
                continue;
            };
            for cp in &manifold.points {
                let mut c =
                    ContactConstraint::new(manifold.a, manifold.b, manifold.normal, cp, &**a, &**b);
                // Warm start: restore cached impulses
                let key = CacheKey::new(c.index_a, c.index_b, c.anchor_a);
                if let Some(&(jn, jt)) = self.cache.get(&key) {
                    c.jn = jn;
                    c.jt = jt;
                }
                self.constraints.push(c);
            }
        }
    }

    /// TGS-style solve: multiple iterations with bias, then restitution pass.
    pub fn solve(&mut self, entities: &mut [Box<dyn PhysicalEntity>]) {
        let dt = if self.inv_dt > 0.0 {
            1.0 / self.inv_dt
        } else {
            0.0
        };

        // Warm start
        for c in &self.constraints {
            c.apply_warm_start(entities);
        }

        // After warm start velocities changed; refresh predicted deltas.
        update_predicted_deltas(entities, dt);

        // Main iterations with bias (corrects penetration)
        for _ in 0..self.iterations {
            // As velocities change during iterations, keep predicted deltas in sync.
            update_predicted_deltas(entities, dt);
            for c in &mut self.constraints {
                c.solve_normal(entities, self.inv_dt, &self.params, true);
            }
            for c in &mut self.constraints {
                c.solve_tangent(entities, self.params.friction);
            }
        }

        // Restitution pass (separate, like Box2D)
        for c in &mut self.constraints {
            c.apply_restitution(
                entities,
                self.params.restitution,
                self.params.restitution_threshold,
            );
        }
    }
}

fn update_predicted_deltas(entities: &mut [Box<dyn PhysicalEntity>], dt: f32) {
    if dt <= 0.0 {
        for e in entities {
            e.clear_deltas();
        }
        return;
    }
    for e in entities {
        *e.delta_pos_mut() = *e.vel() * dt;
        *e.delta_angle_mut() = e.omega() * dt;
    }
}
