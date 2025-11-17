use std::any::Any;

use super::entity::PhysicalEntity;
use super::particle::Particle;
use super::rigid_body::RigidBody;
use super::shape::Collider2D;
use crate::math::mat::Mat2;
use crate::math::vec::Vec2;

#[derive(Debug, Clone)]
pub struct ContactPoint {
    pub point: Vec2,
    pub penetration: f32,
}

#[derive(Debug)]
pub struct Manifold {
    pub a: usize,
    pub b: usize,
    pub normal: Vec2,
    pub tangent: Vec2,
    pub points: Vec<ContactPoint>,
}

impl Manifold {
    fn new(a: usize, b: usize, normal: Vec2, points: Vec<ContactPoint>) -> Self {
        let tangent = normal.perp();
        Self {
            a,
            b,
            normal,
            tangent,
            points,
        }
    }
}

fn extract_collider(entity: &dyn PhysicalEntity) -> Option<(&Collider2D, f32)> {
    let any: &dyn Any = entity;

    if let Some(rb) = any.downcast_ref::<RigidBody>() {
        rb.collider.as_ref().map(|col| (col, rb.angle()))
    } else if any.downcast_ref::<Particle>().is_some() {
        None
    } else {
        None
    }
}

fn circle_circle(
    center_a: &Vec2,
    ra: f32,
    center_b: &Vec2,
    rb: f32,
) -> Option<(Vec2, ContactPoint)> {
    let delta = center_b - center_a;
    let dist_sq = delta.length_squared();
    let radius_sum = ra + rb;

    if dist_sq > radius_sum * radius_sum {
        return None;
    }

    let (normal, penetration) = if let Some(n) = delta.try_normalize() {
        (n, radius_sum - dist_sq.sqrt())
    } else {
        (Vec2::new(1.0, 0.0), radius_sum)
    };

    let contact_point = center_a + &(ra * &normal);
    Some((
        normal,
        ContactPoint {
            point: contact_point,
            penetration,
        },
    ))
}

fn clamp(x: f32, lo: f32, hi: f32) -> f32 {
    x.max(lo).min(hi)
}

fn box_circle(
    box_center: &Vec2,
    box_angle: f32,
    half_extents: &Vec2,
    circ_center: &Vec2,
    radius: f32,
) -> Option<(Vec2, ContactPoint)> {
    let rot = Mat2::rotation(box_angle);
    let inv_rot = rot.transpose();
    let delta_world = circ_center - box_center;
    let delta_local = inv_rot.mul_vec2(&delta_world);

    let closest_local = Vec2::new(
        clamp(delta_local.x, -half_extents.x, half_extents.x),
        clamp(delta_local.y, -half_extents.y, half_extents.y),
    );

    let diff = delta_local - &closest_local;
    let dist_sq = diff.length_squared();

    if dist_sq > radius * radius {
        return None;
    }

    let (normal_local, contact_local, penetration) = if dist_sq > 1e-12 {
        let dist = dist_sq.sqrt();
        let normal_local = diff / dist;
        (normal_local, closest_local, radius - dist)
    } else {
        let dx = half_extents.x - delta_local.x.abs();
        let dy = half_extents.y - delta_local.y.abs();

        if dx < dy {
            let sign_x = delta_local.x.signum();
            let normal_local = Vec2::new(sign_x, 0.0);
            let contact_local = Vec2::new(sign_x * half_extents.x, delta_local.y);
            (normal_local, contact_local, radius + dx)
        } else {
            let sign_y = delta_local.y.signum();
            let normal_local = Vec2::new(0.0, sign_y);
            let contact_local = Vec2::new(delta_local.x, sign_y * half_extents.y);
            (normal_local, contact_local, radius + dy)
        }
    };

    let normal_world = rot.mul_vec2(&normal_local);
    let contact_world = rot.mul_vec2(&contact_local) + box_center;

    Some((
        normal_world,
        ContactPoint {
            point: contact_world,
            penetration,
        },
    ))
}

fn obb_axes(angle: f32) -> (Vec2, Vec2) {
    let (c, s) = (angle.cos(), angle.sin());
    (Vec2::new(c, s), Vec2::new(-s, c))
}

fn project_radius_on_axis(ax: &Vec2, ay: &Vec2, half_extents: &Vec2, axis: &Vec2) -> f32 {
    axis.dot(ax).abs() * half_extents.x + axis.dot(ay).abs() * half_extents.y
}

fn support_point(
    center: &Vec2,
    axes: &(Vec2, Vec2),
    half_extents: &Vec2,
    direction: &Vec2,
) -> Vec2 {
    let (ax, ay) = axes;
    let sign_x = ax.dot(direction).signum();
    let sign_y = ay.dot(direction).signum();
    center + &(sign_x * half_extents.x * ax) + &(sign_y * half_extents.y * ay)
}

fn box_box(
    a_center: &Vec2,
    a_angle: f32,
    a_he: &Vec2,
    b_center: &Vec2,
    b_angle: f32,
    b_he: &Vec2,
) -> Option<(Vec2, ContactPoint)> {
    let a_axes = obb_axes(a_angle);
    let b_axes = obb_axes(b_angle);
    let test_axes = [&a_axes.0, &a_axes.1, &b_axes.0, &b_axes.1];

    let ab = b_center - a_center;
    let mut min_overlap = f32::INFINITY;
    let mut min_axis = Vec2::new(1.0, 0.0);

    for axis in test_axes {
        let ra = project_radius_on_axis(&a_axes.0, &a_axes.1, a_he, axis);
        let rb = project_radius_on_axis(&b_axes.0, &b_axes.1, b_he, axis);
        let center_dist = ab.dot(axis).abs();
        let overlap = ra + rb - center_dist;

        if overlap <= 0.0 {
            return None;
        }

        if overlap < min_overlap {
            min_overlap = overlap;
            let sign = ab.dot(axis).signum();
            min_axis = axis * sign;
        }
    }

    let support_a = support_point(a_center, &a_axes, a_he, &min_axis);
    let support_b = support_point(b_center, &b_axes, b_he, &-&min_axis);
    let contact = (support_a + &support_b) * 0.5;

    Some((
        min_axis,
        ContactPoint {
            point: contact,
            penetration: min_overlap,
        },
    ))
}

fn build_manifold_for_pair(
    a_idx: usize,
    b_idx: usize,
    a: &dyn PhysicalEntity,
    b: &dyn PhysicalEntity,
) -> Option<Manifold> {
    let (shape_a, angle_a) = extract_collider(a)?;
    let (shape_b, angle_b) = extract_collider(b)?;

    let (normal, contact) = match (shape_a, shape_b) {
        (Collider2D::Circle { radius: ra }, Collider2D::Circle { radius: rb }) => {
            circle_circle(a.pos(), *ra, b.pos(), *rb)?
        }
        (Collider2D::Box { half_extents: he }, Collider2D::Circle { radius: r }) => {
            box_circle(a.pos(), angle_a, he, b.pos(), *r)?
        }
        (Collider2D::Circle { radius: r }, Collider2D::Box { half_extents: he }) => {
            let (n, cp) = box_circle(b.pos(), angle_b, he, a.pos(), *r)?;
            (-n, cp)
        }
        (Collider2D::Box { half_extents: hea }, Collider2D::Box { half_extents: heb }) => {
            box_box(a.pos(), angle_a, hea, b.pos(), angle_b, heb)?
        }
    };

    Some(Manifold::new(a_idx, b_idx, normal, vec![contact]))
}

pub fn narrow_phase(
    entities: &Vec<Box<dyn PhysicalEntity>>,
    pairs: &[(usize, usize)],
) -> Vec<Manifold> {
    pairs
        .iter()
        .filter_map(|&(ia, ib)| {
            let a = entities.get(ia)?;
            let b = entities.get(ib)?;
            build_manifold_for_pair(ia, ib, &**a, &**b)
        })
        .collect()
}
