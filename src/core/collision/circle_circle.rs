use super::manifold::ContactPoint;
use crate::math::vec::Vec2;

pub fn detect(
    center_a: Vec2,
    radius_a: f32,
    center_b: Vec2,
    radius_b: f32,
    speculative_distance: f32,
) -> Option<(Vec2, ContactPoint)> {
    let delta = center_b - center_a;
    let dist_sq = delta.length_squared();
    let radius_sum = radius_a + radius_b;

    let max_dist = radius_sum + speculative_distance;
    if dist_sq > max_dist * max_dist {
        return None;
    }

    let (normal, penetration) = delta
        .try_normalize()
        .map(|n| (n, radius_sum - dist_sq.sqrt())) // penetration can be negative => separation
        .unwrap_or((Vec2::new(1.0, 0.0), radius_sum));

    let contact_point = center_a + normal * radius_a;
    Some((
        normal,
        ContactPoint {
            point: contact_point,
            penetration,
        },
    ))
}
