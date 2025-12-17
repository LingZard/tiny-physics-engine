use std::collections::HashMap;

use crate::core::body::PhysicalEntity;
use crate::core::collision::{ContactPoint, Manifold};
use crate::math::mat::Mat2;
use crate::math::vec::Vec2;

#[derive(Debug, Clone)]
pub struct ContactConstraint {
    pub index_a: usize,
    pub index_b: usize,
    pub normal: Vec2,
    pub tangent: Vec2,
    pub local_anchor_a: Vec2,
    pub local_anchor_b: Vec2,
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
        // Store anchors in local space so warm-start matching is stable under rotation.
        let r_a_world0 = cp.point - *a.pos();
        let r_b_world0 = cp.point - *b.pos();
        let rot_a_t = Mat2::rotation(a.angle()).transpose(); // inverse rotation
        let rot_b_t = Mat2::rotation(b.angle()).transpose(); // inverse rotation
        let local_anchor_a = rot_a_t.mul_vec2(r_a_world0);
        let local_anchor_b = rot_b_t.mul_vec2(r_b_world0);

        // World-space anchors at build time (for mass + restitution computation).
        let r_a = Mat2::rotation(a.angle()).mul_vec2(local_anchor_a);
        let r_b = Mat2::rotation(b.angle()).mul_vec2(local_anchor_b);
        let tangent = normal.perp();

        let eff_mass = |axis: Vec2| {
            let rn_a = r_a.cross(axis);
            let rn_b = r_b.cross(axis);
            let inv = a.inv_mass()
                + b.inv_mass()
                + rn_a * rn_a * a.inv_inertia()
                + rn_b * rn_b * b.inv_inertia();
            if inv > 1e-8 { 1.0 / inv } else { 0.0 }
        };

        // Define separation so that at build time (no deltas), separation = -penetration.
        // `penetration` is positive when overlapping, negative when separated.
        let base_separation = -cp.penetration;

        // Save relative velocity for restitution (computed once at constraint build time)
        let rel_vel = velocity_at(r_b, b) - velocity_at(r_a, a);
        let relative_velocity = rel_vel.dot(normal);

        Self {
            index_a,
            index_b,
            normal,
            tangent,
            local_anchor_a,
            local_anchor_b,
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
        delta_pos: &mut [Vec2],
        delta_angle: &mut [f32],
        dt: f32,
        params: &SolverParams,
        use_bias: bool,
    ) {
        let Some((a, b)) = get_pair_mut(entities, self.index_a, self.index_b) else {
            return;
        };

        // World-space anchors at the start of the step from local anchors.
        let r_a0 = Mat2::rotation(a.angle()).mul_vec2(self.local_anchor_a);
        let r_b0 = Mat2::rotation(b.angle()).mul_vec2(self.local_anchor_b);

        // Keep separation linearization consistent with the velocity Jacobian:
        // r(θ + dθ) ≈ r(θ) + dθ * perp(r(θ))
        let dr_a = r_a0.perp() * delta_angle[self.index_a];
        let dr_b = r_b0.perp() * delta_angle[self.index_b];

        // Predicted separation along normal.
        let dp = delta_pos[self.index_b] - delta_pos[self.index_a];
        let separation = self.base_separation + (dp + dr_b - dr_a).dot(self.normal);

        let velocity_bias = if dt <= 0.0 {
            0.0
        } else if separation > 0.0 {
            separation / dt
        } else if use_bias {
            let c = (separation + params.slop).min(0.0);
            (params.bias_rate * c / dt).max(-params.max_bias_velocity)
        } else {
            0.0
        };

        // Relative normal velocity at contact
        let vn = (velocity_at(r_b0, b) - velocity_at(r_a0, a)).dot(self.normal);

        // Incremental normal impulse
        let impulse = -self.normal_mass * (vn + velocity_bias);

        // Clamp the accumulated impulse
        let jn_old = self.jn;
        self.jn = (jn_old + impulse).max(0.0);
        let delta = self.jn - jn_old;

        apply_impulse_pair(a, b, r_a0, r_b0, self.normal, delta);

        sync_pair_deltas(a, b, self.index_a, self.index_b, delta_pos, delta_angle, dt);
    }

    pub fn solve_tangent(
        &mut self,
        entities: &mut [Box<dyn PhysicalEntity>],
        delta_pos: &mut [Vec2],
        delta_angle: &mut [f32],
        dt: f32,
        friction: f32,
    ) {
        let Some((a, b)) = get_pair_mut(entities, self.index_a, self.index_b) else {
            return;
        };

        let r_a0 = Mat2::rotation(a.angle()).mul_vec2(self.local_anchor_a);
        let r_b0 = Mat2::rotation(b.angle()).mul_vec2(self.local_anchor_b);
        let vt = (velocity_at(r_b0, b) - velocity_at(r_a0, a)).dot(self.tangent);
        let lambda = -self.tangent_mass * vt;

        let max_jt = friction * self.jn;
        let jt_old = self.jt;
        self.jt = (jt_old + lambda).clamp(-max_jt, max_jt);
        let delta = self.jt - jt_old;

        apply_impulse_pair(a, b, r_a0, r_b0, self.tangent, delta);

        sync_pair_deltas(a, b, self.index_a, self.index_b, delta_pos, delta_angle, dt);
    }

    /// Apply restitution impulse (separate pass, like Box2D).
    pub fn apply_restitution(
        &mut self,
        entities: &mut [Box<dyn PhysicalEntity>],
        delta_pos: &mut [Vec2],
        delta_angle: &mut [f32],
        dt: f32,
        restitution: f32,
        threshold: f32,
    ) {
        if restitution == 0.0 {
            return;
        }
        if self.base_separation >= 0.0 {
            return;
        }
        if self.relative_velocity > -threshold || self.jn == 0.0 {
            return;
        }

        let Some((a, b)) = get_pair_mut(entities, self.index_a, self.index_b) else {
            return;
        };

        let r_a0 = Mat2::rotation(a.angle()).mul_vec2(self.local_anchor_a);
        let r_b0 = Mat2::rotation(b.angle()).mul_vec2(self.local_anchor_b);
        let vn = (velocity_at(r_b0, b) - velocity_at(r_a0, a)).dot(self.normal);
        let impulse = -self.normal_mass * (vn + restitution * self.relative_velocity);

        let jn_old = self.jn;
        self.jn = (jn_old + impulse).max(0.0);
        let delta = self.jn - jn_old;

        apply_impulse_pair(a, b, r_a0, r_b0, self.normal, delta);

        sync_pair_deltas(a, b, self.index_a, self.index_b, delta_pos, delta_angle, dt);
    }

    fn apply_warm_start(&self, entities: &mut [Box<dyn PhysicalEntity>]) {
        if self.jn == 0.0 && self.jt == 0.0 {
            return;
        }
        let Some((a, b)) = get_pair_mut(entities, self.index_a, self.index_b) else {
            return;
        };
        let r_a0 = Mat2::rotation(a.angle()).mul_vec2(self.local_anchor_a);
        let r_b0 = Mat2::rotation(b.angle()).mul_vec2(self.local_anchor_b);
        apply_impulse_pair(a, b, r_a0, r_b0, self.normal, self.jn);
        apply_impulse_pair(a, b, r_a0, r_b0, self.tangent, self.jt);
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

#[inline]
fn sync_pair_deltas(
    a: &dyn PhysicalEntity,
    b: &dyn PhysicalEntity,
    index_a: usize,
    index_b: usize,
    delta_pos: &mut [Vec2],
    delta_angle: &mut [f32],
    dt: f32,
) {
    if dt <= 0.0 {
        return;
    }
    delta_pos[index_a] = *a.vel() * dt;
    delta_angle[index_a] = a.omega() * dt;
    delta_pos[index_b] = *b.vel() * dt;
    delta_angle[index_b] = b.omega() * dt;
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
    cell_a: (i32, i32),
    cell_b: (i32, i32),
}

impl CacheKey {
    fn cell(v: Vec2) -> (i32, i32) {
        (
            (v.x / CELL_SIZE).round() as i32,
            (v.y / CELL_SIZE).round() as i32,
        )
    }

    fn new(index_a: usize, index_b: usize, local_anchor_a: Vec2, local_anchor_b: Vec2) -> Self {
        Self {
            pair: (index_a, index_b),
            cell_a: Self::cell(local_anchor_a),
            cell_b: Self::cell(local_anchor_b),
        }
    }
}

/// TGS-style solver parameters.
#[derive(Clone)]
pub struct SolverParams {
    /// Baumgarte factor (0.1-0.3 typical). bias = bias_rate * penetration / dt
    pub bias_rate: f32,
    /// Penetration slop (meters). Small penetrations below this do not get corrected by bias.
    pub slop: f32,
    /// Maximum bias velocity (m/s) to prevent explosive corrections
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
        Self {
            // Baumgarte factor: bias = bias_rate * penetration / dt
            // 0.2 is a common value (Box2D uses similar)
            bias_rate: 0.05,
            slop: 0.01,
            // Limit correction speed to prevent explosive behavior
            max_bias_velocity: 4.0,
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
    dt: f32,
    last_dt: f32,
    // Solver-internal predicted per-body deltas for the current step.
    delta_pos: Vec<Vec2>,
    delta_angle: Vec<f32>,
}

impl ConstraintSolver {
    pub fn new(iterations: usize) -> Self {
        Self {
            constraints: Vec::new(),
            iterations,
            params: SolverParams::default(),
            cache: HashMap::new(),
            dt: 0.0,
            last_dt: 0.0,
            delta_pos: Vec::new(),
            delta_angle: Vec::new(),
        }
    }

    pub fn build_constraints(
        &mut self,
        manifolds: &[Manifold],
        entities: &[Box<dyn PhysicalEntity>],
        dt: f32,
    ) {
        self.dt = dt;
        self.ensure_delta_capacity(entities.len());
        let dt_ratio = if self.last_dt > 0.0 {
            dt / self.last_dt
        } else {
            1.0
        };

        // Cache old impulses for warm starting
        self.cache.clear();
        for c in &self.constraints {
            if c.jn != 0.0 || c.jt != 0.0 {
                let key = CacheKey::new(c.index_a, c.index_b, c.local_anchor_a, c.local_anchor_b);
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
                let key = CacheKey::new(c.index_a, c.index_b, c.local_anchor_a, c.local_anchor_b);
                if let Some(&(jn, jt)) = self.cache.get(&key) {
                    // Impulses scale roughly with dt; keep warm-start stable under variable time steps.
                    c.jn = jn * dt_ratio;
                    c.jt = jt * dt_ratio;
                }
                self.constraints.push(c);
            }
        }

        self.last_dt = dt;
    }

    /// TGS-style solve: multiple iterations with bias, then restitution pass.
    pub fn solve(&mut self, entities: &mut [Box<dyn PhysicalEntity>]) {
        let dt = self.dt;

        // Warm start
        for c in &self.constraints {
            c.apply_warm_start(entities);
        }

        // After warm start velocities changed; initialize predicted deltas.
        self.init_predicted_deltas(entities, dt);

        // Main iterations with bias (corrects penetration).
        // Deltas are kept in sync per-body inside solve_* after each impulse.
        for _ in 0..self.iterations {
            for c in &mut self.constraints {
                c.solve_normal(
                    entities,
                    &mut self.delta_pos,
                    &mut self.delta_angle,
                    dt,
                    &self.params,
                    true,
                );
            }
            for c in &mut self.constraints {
                c.solve_tangent(
                    entities,
                    &mut self.delta_pos,
                    &mut self.delta_angle,
                    dt,
                    self.params.friction,
                );
            }
        }

        for c in &mut self.constraints {
            c.apply_restitution(
                entities,
                &mut self.delta_pos,
                &mut self.delta_angle,
                dt,
                self.params.restitution,
                self.params.restitution_threshold,
            );
        }
    }

    #[inline]
    fn ensure_delta_capacity(&mut self, count: usize) {
        if self.delta_pos.len() != count {
            self.delta_pos.resize(count, Vec2::zero());
            self.delta_angle.resize(count, 0.0);
        }
    }

    #[inline]
    fn init_predicted_deltas(&mut self, entities: &[Box<dyn PhysicalEntity>], dt: f32) {
        self.ensure_delta_capacity(entities.len());
        if dt <= 0.0 {
            for d in &mut self.delta_pos {
                *d = Vec2::zero();
            }
            for a in &mut self.delta_angle {
                *a = 0.0;
            }
            return;
        }
        for (i, e) in entities.iter().enumerate() {
            self.delta_pos[i] = *e.vel() * dt;
            self.delta_angle[i] = e.omega() * dt;
        }
    }
}
