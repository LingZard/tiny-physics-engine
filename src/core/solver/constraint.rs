use std::collections::HashMap;

use crate::core::body::PhysicalEntity;
use crate::core::collision::{ContactPoint, Manifold};
use crate::math::vec::Vec2;

#[derive(Debug, Clone)]
pub struct ContactConstraint {
    pub index_a: usize,
    pub index_b: usize,
    pub point: Vec2,
    pub normal: Vec2,
    pub tangent: Vec2,
    pub penetration: f32,
    pub r_a: Vec2,
    pub r_b: Vec2,
    pub normal_mass: f32,
    pub tangent_mass: f32,
    pub jn: f32,
    pub jt: f32,
    pub bias: f32,
    pub friction: f32,
}

impl ContactConstraint {
    fn new(
        index_a: usize,
        index_b: usize,
        normal: Vec2,
        cp: &ContactPoint,
        a: &dyn PhysicalEntity,
        b: &dyn PhysicalEntity,
        params: &SolverParams,
        inv_dt: f32,
    ) -> Self {
        let r_a = cp.point - *a.pos();
        let r_b = cp.point - *b.pos();
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

        let penetration_bias = if cp.penetration > params.slop {
            params.baumgarte * inv_dt * (cp.penetration - params.slop)
        } else {
            0.0
        };

        let rel_vel = velocity_at(r_b, b) - velocity_at(r_a, a);
        let vn = rel_vel.dot(normal);
        let restitution_bias = if vn < -1.0 {
            -params.restitution * vn
        } else {
            0.0
        };

        Self {
            index_a,
            index_b,
            point: cp.point,
            normal,
            tangent,
            penetration: cp.penetration,
            r_a,
            r_b,
            normal_mass: eff_mass(normal),
            tangent_mass: eff_mass(tangent),
            jn: 0.0,
            jt: 0.0,
            bias: penetration_bias + restitution_bias,
            friction: params.friction,
        }
    }

    pub fn solve_normal(&mut self, entities: &mut [Box<dyn PhysicalEntity>]) {
        let Some((a, b)) = get_pair_mut(entities, self.index_a, self.index_b) else {
            return;
        };

        let vn = (velocity_at(self.r_b, b) - velocity_at(self.r_a, a)).dot(self.normal);
        let lambda = -self.normal_mass * (vn - self.bias);

        let jn_old = self.jn;
        self.jn = (jn_old + lambda).max(0.0);
        let delta = self.jn - jn_old;

        apply_impulse_pair(a, b, self.r_a, self.r_b, self.normal, delta);
    }

    pub fn solve_tangent(&mut self, entities: &mut [Box<dyn PhysicalEntity>]) {
        let Some((a, b)) = get_pair_mut(entities, self.index_a, self.index_b) else {
            return;
        };

        let vt = (velocity_at(self.r_b, b) - velocity_at(self.r_a, a)).dot(self.tangent);
        let lambda = -self.tangent_mass * vt;

        let max_jt = self.friction * self.jn;
        let jt_old = self.jt;
        self.jt = (jt_old + lambda).clamp(-max_jt, max_jt);
        let delta = self.jt - jt_old;

        apply_impulse_pair(a, b, self.r_a, self.r_b, self.tangent, delta);
    }

    fn apply_warm_start(&self, entities: &mut [Box<dyn PhysicalEntity>]) {
        if self.jn == 0.0 && self.jt == 0.0 {
            return;
        }
        let Some((a, b)) = get_pair_mut(entities, self.index_a, self.index_b) else {
            return;
        };
        apply_impulse_pair(a, b, self.r_a, self.r_b, self.normal, self.jn);
        apply_impulse_pair(a, b, self.r_a, self.r_b, self.tangent, self.jt);
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
    fn new(c: &ContactConstraint) -> Self {
        Self {
            pair: (c.index_a, c.index_b),
            cell: (
                (c.point.x / CELL_SIZE).round() as i32,
                (c.point.y / CELL_SIZE).round() as i32,
            ),
        }
    }
}

#[derive(Clone)]
pub struct SolverParams {
    pub baumgarte: f32,
    pub slop: f32,
    pub restitution: f32,
    pub friction: f32,
}

impl Default for SolverParams {
    fn default() -> Self {
        Self {
            baumgarte: 0.2,
            slop: 0.01,
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
}

impl ConstraintSolver {
    pub fn new(iterations: usize) -> Self {
        Self {
            constraints: Vec::new(),
            iterations,
            params: SolverParams::default(),
            cache: HashMap::new(),
        }
    }

    pub fn build_constraints(
        &mut self,
        manifolds: &[Manifold],
        entities: &[Box<dyn PhysicalEntity>],
        dt: f32,
    ) {
        self.cache.clear();
        for c in &self.constraints {
            if c.jn != 0.0 || c.jt != 0.0 {
                self.cache.insert(CacheKey::new(c), (c.jn, c.jt));
            }
        }

        self.constraints.clear();
        let inv_dt = if dt > 0.0 { 1.0 / dt } else { 0.0 };

        for manifold in manifolds {
            let (Some(a), Some(b)) = (entities.get(manifold.a), entities.get(manifold.b)) else {
                continue;
            };
            for cp in &manifold.points {
                let mut c = ContactConstraint::new(
                    manifold.a,
                    manifold.b,
                    manifold.normal,
                    cp,
                    &**a,
                    &**b,
                    &self.params,
                    inv_dt,
                );
                if let Some(&(jn, jt)) = self.cache.get(&CacheKey::new(&c)) {
                    c.jn = jn;
                    c.jt = jt;
                }
                self.constraints.push(c);
            }
        }
    }

    pub fn solve(&mut self, entities: &mut [Box<dyn PhysicalEntity>]) {
        for c in &self.constraints {
            c.apply_warm_start(entities);
        }
        for _ in 0..self.iterations {
            for c in &mut self.constraints {
                c.solve_normal(entities);
            }
            for c in &mut self.constraints {
                c.solve_tangent(entities);
            }
        }
    }
}
