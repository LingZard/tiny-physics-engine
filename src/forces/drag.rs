use crate::core::World;

use super::ForceGen;

pub struct LinearDrag {
    pub k: f32, // F = -k v
}

impl ForceGen for LinearDrag {
    fn apply(&self, world: &mut World) {
        for e in world.entities.iter_mut() {
            if e.inv_mass() == 0.0 {
                continue;
            }
            let f = e.force() + &(-e.vel() * self.k);
            *e.force_mut() = f;
        }
    }
}
