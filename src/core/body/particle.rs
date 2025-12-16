use super::entity::PhysicalEntity;
use crate::math::vec::Vec2;

pub struct Particle {
    pos: Vec2,
    vel: Vec2,
    force: Vec2,
    inv_mass: f32,
    angle: f32,
    omega: f32,
    torque: f32,
    inv_inertia: f32,
    delta_pos: Vec2,
    delta_angle: f32,
}

impl Particle {
    pub fn new(pos: Vec2, vel: Vec2, inv_mass: f32) -> Self {
        Self {
            pos,
            vel,
            force: Vec2::zero(),
            inv_mass,
            angle: 0.0,
            omega: 0.0,
            torque: 0.0,
            inv_inertia: 0.0,
            delta_pos: Vec2::zero(),
            delta_angle: 0.0,
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
    fn angle(&self) -> f32 {
        self.angle
    }
    fn angle_mut(&mut self) -> &mut f32 {
        &mut self.angle
    }
    fn omega(&self) -> f32 {
        self.omega
    }
    fn omega_mut(&mut self) -> &mut f32 {
        &mut self.omega
    }
    fn torque(&self) -> f32 {
        self.torque
    }
    fn torque_mut(&mut self) -> &mut f32 {
        &mut self.torque
    }
    fn inv_inertia(&self) -> f32 {
        self.inv_inertia
    }
    fn delta_pos(&self) -> &Vec2 {
        &self.delta_pos
    }
    fn delta_pos_mut(&mut self) -> &mut Vec2 {
        &mut self.delta_pos
    }
    fn delta_angle(&self) -> f32 {
        self.delta_angle
    }
    fn delta_angle_mut(&mut self) -> &mut f32 {
        &mut self.delta_angle
    }
}
