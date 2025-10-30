use super::entity::PhysicalEntity;
use crate::math::vec::Vec2;

pub struct Particle {
    pos: Vec2,
    vel: Vec2,
    force: Vec2,
    inv_mass: f32,
}

impl Particle {
    pub fn new(pos: Vec2, vel: Vec2, inv_mass: f32) -> Self {
        Self {
            pos,
            vel,
            force: Vec2::zero(),
            inv_mass,
        }
    }
}

impl PhysicalEntity for Particle {
    fn pos(&self) -> &Vec2 {
        &self.pos
    }
    fn pos_mut(&mut self) -> &mut Vec2 {
        &mut self.pos
    }
    fn vel(&self) -> &Vec2 {
        &self.vel
    }
    fn vel_mut(&mut self) -> &mut Vec2 {
        &mut self.vel
    }
    fn force(&self) -> &Vec2 {
        &self.force
    }
    fn force_mut(&mut self) -> &mut Vec2 {
        &mut self.force
    }
    fn inv_mass(&self) -> f32 {
        self.inv_mass
    }
}
