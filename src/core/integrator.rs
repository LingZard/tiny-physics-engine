use super::body::PhysicalEntity;

#[derive(Clone, Copy)]
pub enum Integrator {
    #[allow(dead_code)]
    ExplicitEuler,
    SemiImplicitEuler,
}

pub fn integrate(entity: &mut dyn PhysicalEntity, dt: f32, integrator: Integrator) {
    let a = entity.force() * entity.inv_mass();
    match integrator {
        Integrator::ExplicitEuler => {
            let new_pos = entity.pos() + entity.vel() * dt;
            let new_vel = entity.vel() + a * dt;
            *entity.pos_mut() = new_pos;
            *entity.vel_mut() = new_vel;
            // rotation
            let alpha = entity.torque() * entity.inv_inertia();
            let new_angle = entity.angle() + entity.omega() * dt;
            let new_omega = entity.omega() + alpha * dt;
            *entity.angle_mut() = new_angle;
            *entity.omega_mut() = new_omega;
        }
        Integrator::SemiImplicitEuler => {
            let new_vel = entity.vel() + a * dt;
            *entity.vel_mut() = new_vel;
            let new_pos = entity.pos() + entity.vel() * dt;
            *entity.pos_mut() = new_pos;
            // rotation
            let alpha = entity.torque() * entity.inv_inertia();
            let new_omega = entity.omega() + alpha * dt;
            *entity.omega_mut() = new_omega;
            let new_angle = entity.angle() + entity.omega() * dt;
            *entity.angle_mut() = new_angle;
        }
    }
    entity.clear_torque();
}

// rotation merged into `integrate`
