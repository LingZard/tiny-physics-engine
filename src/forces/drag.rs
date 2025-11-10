use crate::core::World;

use super::ForceGen;

pub struct LinearDrag {
    pub k: f32, // F = -k v
}

impl ForceGen for LinearDrag {
    fn apply(&self, world: &mut World) {
        for entity in world.entities.iter_mut().filter(|e| e.inv_mass() > 0.0) {
            let new_force = entity.force() + &(-entity.vel() * self.k);
            *entity.force_mut() = new_force;
        }
    }
}
