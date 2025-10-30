use crate::math::vec::Vec2;

pub trait PhysicalEntity {
    fn pos(&self) -> &Vec2;
    fn pos_mut(&mut self) -> &mut Vec2;
    fn vel(&self) -> &Vec2;
    fn vel_mut(&mut self) -> &mut Vec2;
    fn force(&self) -> &Vec2;
    fn force_mut(&mut self) -> &mut Vec2;
    fn inv_mass(&self) -> f32;
    fn clear_forces(&mut self) {
        *self.force_mut() = Vec2::zero();
    }
}
