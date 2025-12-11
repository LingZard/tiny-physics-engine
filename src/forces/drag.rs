use super::ForceGen;
use crate::core::World;

pub struct LinearDrag {
    pub k: f32,
}

impl ForceGen for LinearDrag {
    fn apply(&self, world: &mut World) {
        for entity in world.entities.iter_mut().filter(|e| e.inv_mass() > 0.0) {
            *entity.force_mut() = *entity.force() - *entity.vel() * self.k;
        }
    }
}
