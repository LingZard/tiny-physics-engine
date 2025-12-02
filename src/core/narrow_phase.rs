use super::entity::PhysicalEntity;
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

fn detect_circle_circle(
    center_a: &Vec2,
    radius_a: f32,
    center_b: &Vec2,
    radius_b: f32,
) -> Option<(Vec2, ContactPoint)> {
    let delta = center_b - center_a;
    let dist_sq = delta.length_squared();
    let radius_sum = radius_a + radius_b;

    if dist_sq > radius_sum * radius_sum {
        return None;
    }

    let (normal, penetration) = if let Some(n) = delta.try_normalize() {
        (n, radius_sum - dist_sq.sqrt())
    } else {
        (Vec2::new(1.0, 0.0), radius_sum)
    };

    let contact_point = center_a + &(radius_a * &normal);
    Some((
        normal,
        ContactPoint {
            point: contact_point,
            penetration,
        },
    ))
}

fn detect_box_circle(
    box_center: &Vec2,
    box_angle: f32,
    half_extents: &Vec2,
    circle_center: &Vec2,
    radius: f32,
) -> Option<(Vec2, ContactPoint)> {
    let rot = Mat2::rotation(box_angle);
    let inv_rot = rot.transpose();
    let delta_world = circle_center - box_center;
    let delta_local = inv_rot.mul_vec2(&delta_world);

    let closest_local = Vec2::new(
        delta_local.x.clamp(-half_extents.x, half_extents.x),
        delta_local.y.clamp(-half_extents.y, half_extents.y),
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

fn detect_box_box(
    center_a: &Vec2,
    angle_a: f32,
    half_extents_a: &Vec2,
    center_b: &Vec2,
    angle_b: f32,
    half_extents_b: &Vec2,
) -> Option<(Vec2, ContactPoint)> {
    let compute_axes = |angle: f32| {
        let (c, s) = (angle.cos(), angle.sin());
        (Vec2::new(c, s), Vec2::new(-s, c))
    };

    let projection_radius = |axes: &(Vec2, Vec2), half_extents: &Vec2, axis: &Vec2| {
        axis.dot(&axes.0).abs() * half_extents.x + axis.dot(&axes.1).abs() * half_extents.y
    };

    let support_point = |center: &Vec2, axes: &(Vec2, Vec2), half_extents: &Vec2, dir: &Vec2| {
        let sign_x = axes.0.dot(dir).signum();
        let sign_y = axes.1.dot(dir).signum();
        center + &(sign_x * half_extents.x * &axes.0) + &(sign_y * half_extents.y * &axes.1)
    };

    let axes_a = compute_axes(angle_a);
    let axes_b = compute_axes(angle_b);
    let test_axes = [&axes_a.0, &axes_a.1, &axes_b.0, &axes_b.1];

    let ab = center_b - center_a;
    let mut min_overlap = f32::INFINITY;
    let mut min_axis = Vec2::new(1.0, 0.0);

    for axis in test_axes {
        let radius_a = projection_radius(&axes_a, half_extents_a, axis);
        let radius_b = projection_radius(&axes_b, half_extents_b, axis);
        let center_distance = ab.dot(axis).abs();
        let overlap = radius_a + radius_b - center_distance;

        if overlap <= 0.0 {
            return None;
        }

        if overlap < min_overlap {
            min_overlap = overlap;
            let sign = ab.dot(axis).signum();
            min_axis = axis * sign;
        }
    }

    let support_a = support_point(center_a, &axes_a, half_extents_a, &min_axis);
    let support_b = support_point(center_b, &axes_b, half_extents_b, &-&min_axis);
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
    index_a: usize,
    index_b: usize,
    entity_a: &dyn PhysicalEntity,
    entity_b: &dyn PhysicalEntity,
) -> Option<Manifold> {
    let collider_a = entity_a.collider()?;
    let collider_b = entity_b.collider()?;

    let angle_a = entity_a.angle();
    let angle_b = entity_b.angle();

    let (normal, contact) = match (collider_a, collider_b) {
        (Collider2D::Circle { radius: ra }, Collider2D::Circle { radius: rb }) => {
            detect_circle_circle(entity_a.pos(), *ra, entity_b.pos(), *rb)?
        }
        (Collider2D::Box { half_extents }, Collider2D::Circle { radius }) => detect_box_circle(
            entity_a.pos(),
            angle_a,
            half_extents,
            entity_b.pos(),
            *radius,
        )?,
        (Collider2D::Circle { radius }, Collider2D::Box { half_extents }) => {
            let (n, cp) = detect_box_circle(
                entity_b.pos(),
                angle_b,
                half_extents,
                entity_a.pos(),
                *radius,
            )?;
            (-n, cp)
        }
        (Collider2D::Box { half_extents: hea }, Collider2D::Box { half_extents: heb }) => {
            detect_box_box(entity_a.pos(), angle_a, hea, entity_b.pos(), angle_b, heb)?
        }
    };

    Some(Manifold::new(index_a, index_b, normal, vec![contact]))
}

pub fn narrow_phase(
    entities: &[Box<dyn PhysicalEntity>],
    pairs: &[(usize, usize)],
) -> Vec<Manifold> {
    pairs
        .iter()
        .filter_map(|&(idx_a, idx_b)| {
            let entity_a = entities.get(idx_a)?;
            let entity_b = entities.get(idx_b)?;
            build_manifold_for_pair(idx_a, idx_b, &**entity_a, &**entity_b)
        })
        .collect()
}
