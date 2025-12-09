use crate::math::vec::Vec2;

pub enum Collider2D {
    Circle { radius: f32 },
    Box { half_extents: Vec2 },
}

pub struct Aabb {
    pub min: Vec2,
    pub max: Vec2,
}

impl Aabb {
    pub fn new(min: Vec2, max: Vec2) -> Self {
        Self { min, max }
    }
    pub fn overlaps(&self, other: &Aabb) -> bool {
        !(self.max.x < other.min.x
            || self.min.x > other.max.x
            || self.max.y < other.min.y
            || self.min.y > other.max.y)
    }
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

    pub fn aabb(&self, pos: &Vec2, angle: f32) -> Aabb {
        match self {
            Collider2D::Circle { radius } => {
                let ext = Vec2::new(*radius, *radius);
                Aabb::new(pos - &ext, pos + &ext)
            }
            Collider2D::Box { half_extents } => {
                let c = angle.cos().abs();
                let s = angle.sin().abs();
                let ex = c * half_extents.x + s * half_extents.y;
                let ey = s * half_extents.x + c * half_extents.y;
                let ext = Vec2::new(ex, ey);
                Aabb::new(pos - &ext, pos + &ext)
            }
        }
    }
}
