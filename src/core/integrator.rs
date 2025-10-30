use super::entity::PhysicalEntity;

#[derive(Clone, Copy)]
pub enum Integrator {
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
        }
        Integrator::SemiImplicitEuler => {
            let new_vel = entity.vel() + a * dt;
            *entity.vel_mut() = new_vel;
            let new_pos = entity.pos() + entity.vel() * dt;
            *entity.pos_mut() = new_pos;
        }
    }
}
