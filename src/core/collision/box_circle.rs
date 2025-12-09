use super::manifold::ContactPoint;
use crate::math::mat::Mat2;
use crate::math::vec::Vec2;

/// Detects collision between a box and a circle.
///
/// Returns the collision normal (pointing from box to circle) and contact point if colliding.
pub fn detect(
    box_center: &Vec2,
    box_angle: f32,
    half_extents: &Vec2,
    circle_center: &Vec2,
    radius: f32,
) -> Option<(Vec2, ContactPoint)> {
    // Transform circle center to box local space
    let rot = Mat2::rotation(box_angle);
    let inv_rot = rot.transpose();
    let delta_world = circle_center - box_center;
    let delta_local = inv_rot.mul_vec2(&delta_world);

    // Find closest point on box to circle center (in local space)
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
        // Circle center is outside the box
        let dist = dist_sq.sqrt();
        let normal_local = diff / dist;
        (normal_local, closest_local, radius - dist)
    } else {
        // Circle center is inside the box - find closest face
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

    // Transform back to world space
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
