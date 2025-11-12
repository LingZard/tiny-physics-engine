use crate::math::vec::Vec2;

pub enum Collider2D {
    Circle { radius: f32 },
    Box { half_extents: Vec2 },
}

impl Collider2D {
    pub fn inertia_about_center(&self, mass: f32) -> f32 {
        if mass <= 0.0 {
            return 0.0;
        }
        match self {
            Collider2D::Circle { radius } => 0.5 * mass * radius * radius,
            Collider2D::Box { half_extents } => {
                let w = half_extents.x * 2.0;
                let h = half_extents.y * 2.0;
                mass * (w * w + h * h) / 12.0
            }
        }
    }
}
