use std::ops::{Add, Mul, Sub};

use super::vec::Vec2;

/// 2x2 matrix in row-major layout:
/// [ m00 m01 ]
/// [ m10 m11 ]
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
        // [ c -s ]
        // [ s  c ]
        Self::new(c, -s, s, c)
    }

    pub fn scaling(sx: f32, sy: f32) -> Self {
        // [ sx  0 ]
        // [  0 sy ]
        Self::new(sx, 0.0, 0.0, sy)
    }

    pub fn transpose(&self) -> Self {
        Self::new(self.m00, self.m10, self.m01, self.m11)
    }

    pub fn determinant(&self) -> f32 {
        self.m00 * self.m11 - self.m01 * self.m10
    }

    pub fn inverse(&self) -> Option<Self> {
        let det = self.determinant();
        if det == 0.0 {
            return None;
        }
        let inv_det = 1.0 / det;
        // 1/det * [  m11  -m01 ]
        //         [ -m10   m00 ]
        Some(Self::new(
            self.m11 * inv_det,
            -self.m01 * inv_det,
            -self.m10 * inv_det,
            self.m00 * inv_det,
        ))
    }

    pub fn mul_vec2(&self, v: &Vec2) -> Vec2 {
        Vec2::new(
            self.m00 * v.x + self.m01 * v.y,
            self.m10 * v.x + self.m11 * v.y,
        )
    }

    pub fn mul_mat2(&self, rhs: &Mat2) -> Mat2 {
        // self * rhs
        Mat2::new(
            self.m00 * rhs.m00 + self.m01 * rhs.m10,
            self.m00 * rhs.m01 + self.m01 * rhs.m11,
            self.m10 * rhs.m00 + self.m11 * rhs.m10,
            self.m10 * rhs.m01 + self.m11 * rhs.m11,
        )
    }
}

// Matrix + Matrix
impl<'a, 'b> Add<&'b Mat2> for &'a Mat2 {
    type Output = Mat2;

    fn add(self, rhs: &'b Mat2) -> Mat2 {
        Mat2::new(
            self.m00 + rhs.m00,
            self.m01 + rhs.m01,
            self.m10 + rhs.m10,
            self.m11 + rhs.m11,
        )
    }
}

impl Add<&Mat2> for Mat2 {
    type Output = Mat2;

    fn add(self, rhs: &Mat2) -> Mat2 {
        &self + rhs
    }
}

impl Add<Mat2> for &Mat2 {
    type Output = Mat2;

    fn add(self, rhs: Mat2) -> Mat2 {
        self + &rhs
    }
}

impl Add<Mat2> for Mat2 {
    type Output = Mat2;

    fn add(self, rhs: Mat2) -> Mat2 {
        &self + &rhs
    }
}

// Matrix - Matrix
impl<'a, 'b> Sub<&'b Mat2> for &'a Mat2 {
    type Output = Mat2;

    fn sub(self, rhs: &'b Mat2) -> Mat2 {
        Mat2::new(
            self.m00 - rhs.m00,
            self.m01 - rhs.m01,
            self.m10 - rhs.m10,
            self.m11 - rhs.m11,
        )
    }
}

impl Sub<&Mat2> for Mat2 {
    type Output = Mat2;

    fn sub(self, rhs: &Mat2) -> Mat2 {
        &self - rhs
    }
}

impl Sub<Mat2> for &Mat2 {
    type Output = Mat2;

    fn sub(self, rhs: Mat2) -> Mat2 {
        self - &rhs
    }
}

impl Sub<Mat2> for Mat2 {
    type Output = Mat2;

    fn sub(self, rhs: Mat2) -> Mat2 {
        &self - &rhs
    }
}

// Matrix * Vector
impl<'a, 'b> Mul<&'b Vec2> for &'a Mat2 {
    type Output = Vec2;

    fn mul(self, rhs: &'b Vec2) -> Vec2 {
        self.mul_vec2(rhs)
    }
}

impl Mul<&Vec2> for Mat2 {
    type Output = Vec2;

    fn mul(self, rhs: &Vec2) -> Vec2 {
        &self * rhs
    }
}

impl Mul<Vec2> for &Mat2 {
    type Output = Vec2;

    fn mul(self, rhs: Vec2) -> Vec2 {
        self * &rhs
    }
}

impl Mul<Vec2> for Mat2 {
    type Output = Vec2;

    fn mul(self, rhs: Vec2) -> Vec2 {
        &self * &rhs
    }
}

// Matrix * Matrix
impl<'a, 'b> Mul<&'b Mat2> for &'a Mat2 {
    type Output = Mat2;

    fn mul(self, rhs: &'b Mat2) -> Mat2 {
        self.mul_mat2(rhs)
    }
}

impl Mul<&Mat2> for Mat2 {
    type Output = Mat2;

    fn mul(self, rhs: &Mat2) -> Mat2 {
        &self * rhs
    }
}

impl Mul<Mat2> for &Mat2 {
    type Output = Mat2;

    fn mul(self, rhs: Mat2) -> Mat2 {
        self * &rhs
    }
}

impl Mul<Mat2> for Mat2 {
    type Output = Mat2;

    fn mul(self, rhs: Mat2) -> Mat2 {
        &self * &rhs
    }
}
