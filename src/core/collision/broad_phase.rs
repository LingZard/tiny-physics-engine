use std::any::Any;

use super::{Aabb, Collider2D};
use crate::core::body::{Particle, PhysicalEntity, RigidBody};
use crate::math::vec::Vec2;

/// Computes the AABB for an entity.
fn entity_aabb(e: &dyn PhysicalEntity) -> Aabb {
    let any: &dyn Any = e;

    if let Some(rb) = any.downcast_ref::<RigidBody>() {
        if let Some(col) = &rb.collider {
            return col.aabb(rb.pos(), rb.angle());
        } else {
            let ext = Vec2::new(0.05, 0.05);
            let min = rb.pos() - &ext;
            let max = rb.pos() + &ext;
            return Aabb::new(min, max);
        }
    }

    if let Some(p) = any.downcast_ref::<Particle>() {
        let r = 0.05f32;
        let col = Collider2D::Circle { radius: r };
        return col.aabb(p.pos(), p.angle());
    }

    // Fallback for unknown entity types
    let ext = Vec2::new(0.01, 0.01);
    let min = e.pos() - &ext;
    let max = e.pos() + &ext;
    Aabb::new(min, max)
}

/// Performs broad phase collision detection using Sweep and Prune.
///
/// Returns a list of potentially colliding entity index pairs.
pub fn detect_sap(entities: &Vec<Box<dyn PhysicalEntity>>) -> Vec<(usize, usize)> {
    struct Entry {
        index: usize,
        aabb: Aabb,
    }

    // Build sorted entry list
    let mut entries: Vec<Entry> = entities
        .iter()
        .enumerate()
        .map(|(i, e)| Entry {
            index: i,
            aabb: entity_aabb(&**e),
        })
        .collect();

    entries.sort_by(|a, b| {
        a.aabb
            .min
            .x
            .partial_cmp(&b.aabb.min.x)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // Sweep and prune
    let mut active: Vec<Entry> = Vec::new();
    let mut pairs: Vec<(usize, usize)> = Vec::new();

    for cur in entries.into_iter() {
        // Remove entries that can no longer overlap
        active.retain(|e| e.aabb.max.x >= cur.aabb.min.x);

        // Check for overlaps with active entries
        for e in active.iter() {
            if e.aabb.overlaps(&cur.aabb) {
                let (i, j) = if e.index < cur.index {
                    (e.index, cur.index)
                } else {
                    (cur.index, e.index)
                };
                pairs.push((i, j));
            }
        }

        active.push(cur);
    }

    pairs
}
