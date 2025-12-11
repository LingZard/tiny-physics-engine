use super::vec::Vec2;

#[derive(Debug, Clone, Copy)]
pub struct Mat2 {
    pub m00: f32,
    pub m01: f32,
    pub m10: f32,
    pub m11: f32,
}

impl Mat2 {
    pub fn new(m00: f32, m01: f32, m10: f32, m11: f32) -> Self {
        Self { m00, m01, m10, m11 }
    }

    pub fn identity() -> Self {
        Self::new(1.0, 0.0, 0.0, 1.0)
    }

    pub fn rotation(radians: f32) -> Self {
        let c = radians.cos();
        let s = radians.sin();
        Self::new(c, -s, s, c)
    }

    pub fn transpose(self) -> Self {
        Self::new(self.m00, self.m10, self.m01, self.m11)
    }

    pub fn mul_vec2(self, v: Vec2) -> Vec2 {
        Vec2::new(
            self.m00 * v.x + self.m01 * v.y,
            self.m10 * v.x + self.m11 * v.y,
        )
    }

    pub fn mul_mat2(self, rhs: &Mat2) -> Mat2 {
        Mat2::new(
            self.m00 * rhs.m00 + self.m01 * rhs.m10,
            self.m00 * rhs.m01 + self.m01 * rhs.m11,
            self.m10 * rhs.m00 + self.m11 * rhs.m10,
            self.m10 * rhs.m01 + self.m11 * rhs.m11,
        )
    }
}
