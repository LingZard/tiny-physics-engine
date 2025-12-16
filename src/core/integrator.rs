use super::body::PhysicalEntity;

#[derive(Clone, Copy)]
pub enum Integrator {
    #[allow(dead_code)]
    ExplicitEuler,
    SemiImplicitEuler,
}

/// Integrate linear/angular velocity using accumulated force/torque.
///
/// Note: this does NOT clear force/torque; the caller controls accumulator lifetime.
pub fn integrate_velocity(entity: &mut dyn PhysicalEntity, dt: f32, integrator: Integrator) {
    if dt <= 0.0 {
        return;
    }
    let a = *entity.force() * entity.inv_mass();
    let alpha = entity.torque() * entity.inv_inertia();
    match integrator {
        Integrator::ExplicitEuler | Integrator::SemiImplicitEuler => {
            *entity.vel_mut() = *entity.vel() + a * dt;
            *entity.omega_mut() = entity.omega() + alpha * dt;
        }
    }
}

/// Integrate position/orientation from the current velocity.
pub fn integrate_position(entity: &mut dyn PhysicalEntity, dt: f32, _integrator: Integrator) {
    if dt <= 0.0 {
        return;
    }
    *entity.pos_mut() = *entity.pos() + *entity.vel() * dt;
    *entity.angle_mut() = entity.angle() + entity.omega() * dt;
}

pub fn integrate(entity: &mut dyn PhysicalEntity, dt: f32, integrator: Integrator) {
    match integrator {
        Integrator::ExplicitEuler => {
            // Euler: position from old velocity, then update velocity.
            integrate_position(entity, dt, integrator);
            integrate_velocity(entity, dt, integrator);
        }
        Integrator::SemiImplicitEuler => {
            // Symplectic Euler: velocity first, then position from new velocity.
            integrate_velocity(entity, dt, integrator);
            integrate_position(entity, dt, integrator);
        }
    }
}
