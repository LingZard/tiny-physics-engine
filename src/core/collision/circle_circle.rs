use super::manifold::ContactPoint;
use crate::math::vec::Vec2;

/// Detects collision between two circles.
///
/// Returns the collision normal (pointing from A to B) and contact point if colliding.
pub fn detect(
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
        // Circles are at the same position, use arbitrary normal
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
