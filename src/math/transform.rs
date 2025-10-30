use std::ops::Mul;

use super::mat::Mat2;
use super::vec::Vec2;

/// Rigid 2D transform: x' = R x + t
pub struct Transform2D {
    pub rotation: Mat2,
    pub translation: Vec2,
}

impl Transform2D {
    pub fn new(rotation: Mat2, translation: Vec2) -> Self {
        Self {
            rotation,
            translation,
        }
    }

    pub fn identity() -> Self {
        Self::new(Mat2::identity(), Vec2::zero())
    }

    pub fn from_translation(translation: Vec2) -> Self {
        Self::new(Mat2::identity(), translation)
    }

    pub fn from_rotation(radians: f32) -> Self {
        Self::new(Mat2::rotation(radians), Vec2::zero())
    }

    pub fn apply_to_point(&self, p: &Vec2) -> Vec2 {
        &self.rotation * p + &self.translation
    }

    pub fn apply_to_vector(&self, v: &Vec2) -> Vec2 {
        &self.rotation * v
    }

    /// Compose transforms: first apply `self`, then `other`.
    pub fn then(&self, other: &Self) -> Self {
        let rotation = &other.rotation * &self.rotation;
        let translation = &other.rotation * &self.translation + &other.translation;
        Self::new(rotation, translation)
    }

    pub fn inverse(&self) -> Self {
        // For rigid transform, R is orthonormal: R^{-1} = R^T.
        let r_inv = self.rotation.transpose();
        let t_inv = -(&r_inv * &self.translation);
        Self::new(r_inv, t_inv)
    }
}

// Transform * point (treat rhs as a point)
impl<'a, 'b> Mul<&'b Vec2> for &'a Transform2D {
    type Output = Vec2;

    fn mul(self, rhs: &'b Vec2) -> Vec2 {
        self.apply_to_point(rhs)
    }
}

impl Mul<&Vec2> for Transform2D {
    type Output = Vec2;

    fn mul(self, rhs: &Vec2) -> Vec2 {
        &self * rhs
    }
}

impl Mul<Vec2> for &Transform2D {
    type Output = Vec2;

    fn mul(self, rhs: Vec2) -> Vec2 {
        self * &rhs
    }
}

impl Mul<Vec2> for Transform2D {
    type Output = Vec2;

    fn mul(self, rhs: Vec2) -> Vec2 {
        &self * &rhs
    }
}

// Transform * Transform (composition): first self, then rhs
impl<'a, 'b> Mul<&'b Transform2D> for &'a Transform2D {
    type Output = Transform2D;

    fn mul(self, rhs: &'b Transform2D) -> Transform2D {
        self.then(rhs)
    }
}

impl Mul<&Transform2D> for Transform2D {
    type Output = Transform2D;

    fn mul(self, rhs: &Transform2D) -> Transform2D {
        &self * rhs
    }
}

impl Mul<Transform2D> for &Transform2D {
    type Output = Transform2D;

    fn mul(self, rhs: Transform2D) -> Transform2D {
        self * &rhs
    }
}

impl Mul<Transform2D> for Transform2D {
    type Output = Transform2D;

    fn mul(self, rhs: Transform2D) -> Transform2D {
        &self * &rhs
    }
}
