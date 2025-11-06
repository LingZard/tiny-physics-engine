pub mod drag;
pub mod spring;

use std::any::Any;

use crate::core::World;

pub trait ForceGen: Any {
    fn apply(&self, world: &mut World);
}
