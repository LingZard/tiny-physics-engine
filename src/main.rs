mod core;
mod math;

use core::{Integrator, World, particle::Particle};
use math::vec::Vec2;

fn main() {
    let gravity = Vec2::new(0.0, -9.81);
    let mut world = World::new(gravity, Integrator::SemiImplicitEuler);

    let p0 = Vec2::new(0.0, 5.0);
    let v0 = Vec2::zero();
    let particle = Particle::new(p0, v0, 1.0);
    world.add(Box::new(particle));

    let dt = 1.0 / 60.0;
    let steps = 120;

    println!("t, x, y, vx, vy");
    for i in 0..steps {
        let t = i as f32 * dt;
        let e = &world.entities[0];
        let pos = e.pos();
        let vel = e.vel();
        println!(
            "{:.3}, {:.5}, {:.5}, {:.5}, {:.5}",
            t, pos.x, pos.y, vel.x, vel.y
        );

        world.step(dt);
    }
}
