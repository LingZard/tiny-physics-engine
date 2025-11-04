pub mod drag;
pub mod spring;

use crate::core::World;

pub trait ForceGen {
    fn apply(&self, world: &mut World);
}
