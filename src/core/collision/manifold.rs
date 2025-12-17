use crate::math::vec::Vec2;

/// A single contact point in a collision manifold.
#[derive(Debug, Clone)]
pub struct ContactPoint {
    /// World-space position of the contact point.
    pub point: Vec2,
    /// Signed penetration depth.
    /// - **positive**: overlapping (penetration)
    /// - **negative**: separated (speculative contact / separation distance)
    pub penetration: f32,
}

/// Collision manifold containing contact information between two entities.
#[derive(Debug)]
pub struct Manifold {
    /// Index of the first entity.
    pub a: usize,
    /// Index of the second entity.
    pub b: usize,
    /// Collision normal pointing from A to B.
    pub normal: Vec2,
    /// Tangent vector (perpendicular to normal).
    pub tangent: Vec2,
    /// Contact points.
    pub points: Vec<ContactPoint>,
}

impl Manifold {
    pub fn new(a: usize, b: usize, normal: Vec2, points: Vec<ContactPoint>) -> Self {
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
