use super::collision::{ContactPoint, Manifold};
use super::entity::PhysicalEntity;
use crate::math::vec::Vec2;

#[derive(Debug)]
pub struct ContactConstraint {
    pub index_a: usize,
    pub index_b: usize,
    pub normal: Vec2,
    pub tangent: Vec2,
    pub point: Vec2,
    pub penetration: f32,

    pub normal_mass: f32,
    pub tangent_mass: f32,

    pub r_a: Vec2,
    pub r_b: Vec2,

    pub accumulated_normal_impulse: f32,
    pub accumulated_tangent_impulse: f32,

    pub restitution: f32,
    pub friction: f32,

    pub velocity_bias: f32,
}

impl ContactConstraint {
    pub fn from_manifold(
        manifold: &Manifold,
        entities: &[Box<dyn PhysicalEntity>],
        restitution: f32,
        friction: f32,
        baumgarte_factor: f32,
        slop: f32,
        inv_dt: f32,
    ) -> Vec<Self> {
        let entity_a = match entities.get(manifold.a) {
            Some(e) => e,
            None => return vec![],
        };
        let entity_b = match entities.get(manifold.b) {
            Some(e) => e,
            None => return vec![],
        };

        manifold
            .points
            .iter()
            .map(|cp| {
                Self::from_contact_point(
                    manifold.a,
                    manifold.b,
                    &manifold.normal,
                    &manifold.tangent,
                    cp,
                    &**entity_a,
                    &**entity_b,
                    restitution,
                    friction,
                    baumgarte_factor,
                    slop,
                    inv_dt,
                )
            })
            .collect()
    }

    fn from_contact_point(
        index_a: usize,
        index_b: usize,
        normal: &Vec2,
        tangent: &Vec2,
        cp: &ContactPoint,
        entity_a: &dyn PhysicalEntity,
        entity_b: &dyn PhysicalEntity,
        restitution: f32,
        friction: f32,
        baumgarte_factor: f32,
        slop: f32,
        inv_dt: f32,
    ) -> Self {
        let r_a = &cp.point - entity_a.pos();
        let r_b = &cp.point - entity_b.pos();

        let compute_effective_mass = |axis: &Vec2| {
            let cross_a = r_a.x * axis.y - r_a.y * axis.x;
            let cross_b = r_b.x * axis.y - r_b.y * axis.x;
            let inv = entity_a.inv_mass()
                + entity_b.inv_mass()
                + cross_a * cross_a * entity_a.inv_inertia()
                + cross_b * cross_b * entity_b.inv_inertia();
            if inv > 1e-8 { 1.0 / inv } else { 0.0 }
        };

        let normal_mass = compute_effective_mass(normal);
        let tangent_mass = compute_effective_mass(tangent);

        let vel_a =
            entity_a.vel() + &Vec2::new(-entity_a.omega() * r_a.y, entity_a.omega() * r_a.x);
        let vel_b =
            entity_b.vel() + &Vec2::new(-entity_b.omega() * r_b.y, entity_b.omega() * r_b.x);
        let rel_vel = vel_b - &vel_a;
        let vel_along_normal = rel_vel.dot(normal);

        let velocity_bias = if cp.penetration > slop {
            baumgarte_factor * inv_dt * (cp.penetration - slop)
        } else {
            0.0
        };

        let restitution_threshold = 1.0;
        let restitution_bias = if vel_along_normal < -restitution_threshold {
            -restitution * vel_along_normal
        } else {
            0.0
        };

        Self {
            index_a,
            index_b,
            normal: *normal,
            tangent: *tangent,
            point: cp.point,
            penetration: cp.penetration,
            normal_mass,
            tangent_mass,
            r_a,
            r_b,
            accumulated_normal_impulse: 0.0,
            accumulated_tangent_impulse: 0.0,
            restitution,
            friction,
            velocity_bias: velocity_bias + restitution_bias,
        }
    }

    #[inline]
    fn velocity_at_point(e: &dyn PhysicalEntity, r: &Vec2) -> Vec2 {
        e.vel() + &Vec2::new(-e.omega() * r.y, e.omega() * r.x)
    }

    #[inline]
    fn apply_impulse(e: &mut dyn PhysicalEntity, r: &Vec2, dir: &Vec2, magnitude: f32) {
        let impulse = dir * magnitude;
        *e.vel_mut() = e.vel() + &(e.inv_mass() * &impulse);
        let torque = r.x * impulse.y - r.y * impulse.x;
        *e.omega_mut() = e.omega() + e.inv_inertia() * torque;
    }

    pub fn solve_normal(&mut self, entities: &mut [Box<dyn PhysicalEntity>]) {
        let (entity_a, entity_b) = get_entity_pair_mut(entities, self.index_a, self.index_b);
        if entity_a.is_none() || entity_b.is_none() {
            return;
        }
        let (entity_a, entity_b) = (entity_a.unwrap(), entity_b.unwrap());

        let rel_vel = Self::velocity_at_point(entity_b, &self.r_b)
            - &Self::velocity_at_point(entity_a, &self.r_a);
        let vel_along_normal = rel_vel.dot(&self.normal);

        let lambda = -self.normal_mass * (vel_along_normal - self.velocity_bias);

        let old_impulse = self.accumulated_normal_impulse;
        self.accumulated_normal_impulse = (old_impulse + lambda).max(0.0);
        let delta_impulse = self.accumulated_normal_impulse - old_impulse;

        Self::apply_impulse(entity_a, &self.r_a, &self.normal, -delta_impulse);
        Self::apply_impulse(entity_b, &self.r_b, &self.normal, delta_impulse);
    }

    pub fn solve_tangent(&mut self, entities: &mut [Box<dyn PhysicalEntity>]) {
        let (entity_a, entity_b) = get_entity_pair_mut(entities, self.index_a, self.index_b);
        if entity_a.is_none() || entity_b.is_none() {
            return;
        }
        let (entity_a, entity_b) = (entity_a.unwrap(), entity_b.unwrap());

        let rel_vel = Self::velocity_at_point(entity_b, &self.r_b)
            - &Self::velocity_at_point(entity_a, &self.r_a);
        let vel_along_tangent = rel_vel.dot(&self.tangent);

        let lambda = -self.tangent_mass * vel_along_tangent;

        let max_friction = self.friction * self.accumulated_normal_impulse;
        let old_impulse = self.accumulated_tangent_impulse;
        self.accumulated_tangent_impulse =
            (old_impulse + lambda).clamp(-max_friction, max_friction);
        let delta_impulse = self.accumulated_tangent_impulse - old_impulse;

        Self::apply_impulse(entity_a, &self.r_a, &self.tangent, -delta_impulse);
        Self::apply_impulse(entity_b, &self.r_b, &self.tangent, delta_impulse);
    }
}

fn get_entity_pair_mut(
    entities: &mut [Box<dyn PhysicalEntity>],
    idx_a: usize,
    idx_b: usize,
) -> (
    Option<&mut dyn PhysicalEntity>,
    Option<&mut dyn PhysicalEntity>,
) {
    if idx_a == idx_b {
        return (None, None);
    }

    let (left, right) = if idx_a < idx_b {
        let (left, right) = entities.split_at_mut(idx_b);
        (Some(&mut *left[idx_a]), right.get_mut(0).map(|e| &mut **e))
    } else {
        let (left, right) = entities.split_at_mut(idx_a);
        (right.get_mut(0).map(|e| &mut **e), Some(&mut *left[idx_b]))
    };

    (left, right)
}

pub struct ConstraintSolver {
    pub constraints: Vec<ContactConstraint>,
    pub iterations: usize,
    pub baumgarte_factor: f32,
    pub slop: f32,
}

impl ConstraintSolver {
    pub fn new(iterations: usize) -> Self {
        Self {
            constraints: Vec::new(),
            iterations,
            baumgarte_factor: 0.2,
            slop: 0.01,
        }
    }

    pub fn build_constraints(
        &mut self,
        manifolds: &[Manifold],
        entities: &[Box<dyn PhysicalEntity>],
        restitution: f32,
        friction: f32,
        dt: f32,
    ) {
        self.constraints.clear();
        let inv_dt = if dt > 0.0 { 1.0 / dt } else { 0.0 };
        for manifold in manifolds {
            let mut constraints = ContactConstraint::from_manifold(
                manifold,
                entities,
                restitution,
                friction,
                self.baumgarte_factor,
                self.slop,
                inv_dt,
            );
            self.constraints.append(&mut constraints);
        }
    }

    pub fn solve(&mut self, entities: &mut [Box<dyn PhysicalEntity>]) {
        for _ in 0..self.iterations {
            for constraint in &mut self.constraints {
                constraint.solve_normal(entities);
            }
            for constraint in &mut self.constraints {
                constraint.solve_tangent(entities);
            }
        }
    }
}
