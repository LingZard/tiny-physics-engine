use super::PhysicalEntity;
use crate::core::collision::Collider2D;
use crate::math::vec::Vec2;

pub struct RigidBody {
    pub pos: Vec2,
    pub vel: Vec2,
    pub force: Vec2,
    pub inv_mass: f32,
    pub angle: f32,
    pub omega: f32,
    pub torque: f32,
    pub inv_inertia: f32,
    pub collider: Option<Collider2D>,
    // TGS-style: track position/angle change within current step
    pub delta_pos: Vec2,
    pub delta_angle: f32,
}

impl RigidBody {
    pub fn new(pos: Vec2, angle: f32, inv_mass: f32, inv_inertia: f32) -> Self {
        Self {
            pos,
            vel: Vec2::zero(),
            force: Vec2::zero(),
            inv_mass,
            angle,
            omega: 0.0,
            torque: 0.0,
            inv_inertia,
            collider: None,
            delta_pos: Vec2::zero(),
            delta_angle: 0.0,
        }
    }

    pub fn box_xy(pos: Vec2, angle: f32, mass: f32, width: f32, height: f32) -> Self {
        let inv_mass = if mass > 0.0 { 1.0 / mass } else { 0.0 };
        let collider = Collider2D::Box {
            half_extents: Vec2::new(width * 0.5, height * 0.5),
        };
        let inertia = collider.inertia_about_center(mass);
        let inv_inertia = if inertia > 0.0 { 1.0 / inertia } else { 0.0 };
        Self {
            pos,
            vel: Vec2::zero(),
            force: Vec2::zero(),
            inv_mass,
            angle,
            omega: 0.0,
            torque: 0.0,
            inv_inertia,
            collider: Some(collider),
            delta_pos: Vec2::zero(),
            delta_angle: 0.0,
        }
    }

    pub fn circle(pos: Vec2, angle: f32, mass: f32, radius: f32) -> Self {
        let inv_mass = if mass > 0.0 { 1.0 / mass } else { 0.0 };
        let collider = Collider2D::Circle { radius };
        let inertia = collider.inertia_about_center(mass);
        let inv_inertia = if inertia > 0.0 { 1.0 / inertia } else { 0.0 };
        Self {
            pos,
            vel: Vec2::zero(),
            force: Vec2::zero(),
            inv_mass,
            angle,
            omega: 0.0,
            torque: 0.0,
            inv_inertia,
            collider: Some(collider),
            delta_pos: Vec2::zero(),
            delta_angle: 0.0,
        }
    }
}

impl PhysicalEntity for RigidBody {
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
    fn collider(&self) -> Option<&Collider2D> {
        self.collider.as_ref()
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
