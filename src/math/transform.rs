use super::mat::Mat2;
use super::vec::Vec2;

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

    pub fn apply_to_point(&self, p: Vec2) -> Vec2 {
        self.rotation.mul_vec2(p) + self.translation
    }

    pub fn apply_to_vector(&self, v: Vec2) -> Vec2 {
        self.rotation.mul_vec2(v)
    }

    pub fn then(&self, other: &Self) -> Self {
        let rotation = other.rotation.mul_mat2(&self.rotation);
        let translation = other.rotation.mul_vec2(self.translation) + other.translation;
        Self::new(rotation, translation)
    }

    pub fn inverse(&self) -> Self {
        let r_inv = self.rotation.transpose();
        let t_inv = -r_inv.mul_vec2(self.translation);
        Self::new(r_inv, t_inv)
    }
}
