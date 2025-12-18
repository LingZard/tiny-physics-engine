use super::manifold::ContactPoint;
use crate::math::mat::Mat2;
use crate::math::vec::Vec2;

pub fn detect(
    box_center: Vec2,
    box_angle: f32,
    half_extents: Vec2,
    circle_center: Vec2,
    radius: f32,
    speculative_distance: f32,
) -> Option<(Vec2, ContactPoint)> {
    let rot = Mat2::rotation(box_angle);
    let inv_rot = rot.transpose();
    let delta_world = circle_center - box_center;
    let delta_local = inv_rot.mul_vec2(delta_world);

    let closest_local = Vec2::new(
        delta_local.x.clamp(-half_extents.x, half_extents.x),
        delta_local.y.clamp(-half_extents.y, half_extents.y),
    );

    let diff = delta_local - closest_local;
    let dist_sq = diff.length_squared();

    let max_r = radius + speculative_distance;
    if dist_sq > max_r * max_r {
        return None;
    }

    let (normal_local, contact_local, penetration) = if dist_sq > 1e-12 {
        let dist = dist_sq.sqrt();
        // penetration can be negative => separated but within speculative distance
        (diff / dist, closest_local, radius - dist)
    } else {
        let dx = half_extents.x - delta_local.x.abs();
        let dy = half_extents.y - delta_local.y.abs();

        if dx < dy {
            let sign_x = delta_local.x.signum();
            (
                Vec2::new(sign_x, 0.0),
                Vec2::new(sign_x * half_extents.x, delta_local.y),
                radius + dx, // inside box => overlap
            )
        } else {
            let sign_y = delta_local.y.signum();
            (
                Vec2::new(0.0, sign_y),
                Vec2::new(delta_local.x, sign_y * half_extents.y),
                radius + dy, // inside box => overlap
            )
        }
    };

    let normal_world = rot.mul_vec2(normal_local);
    let contact_world = rot.mul_vec2(contact_local) + box_center;

    Some((
        normal_world,
        ContactPoint {
            point: contact_world,
            penetration,
        },
    ))
}
