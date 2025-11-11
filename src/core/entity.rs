use crate::math::vec::Vec2;
use std::any::Any;

pub trait PhysicalEntity: Any {
    // translation
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
    // rotation
    fn angle(&self) -> f32;
    fn angle_mut(&mut self) -> &mut f32;
    fn omega(&self) -> f32;
    fn omega_mut(&mut self) -> &mut f32;
    fn torque(&self) -> f32;
    fn torque_mut(&mut self) -> &mut f32;
    fn inv_inertia(&self) -> f32;
    fn clear_torque(&mut self) {
        *self.torque_mut() = 0.0;
    }
}
