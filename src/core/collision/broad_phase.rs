use std::any::Any;

use super::{Aabb, Collider2D};
use crate::core::body::{Particle, PhysicalEntity, RigidBody};
use crate::math::vec::Vec2;

// Expand AABBs to generate speculative contact pairs (Box2D-style "fat AABB").
// This allows narrow-phase to create contacts slightly before actual overlap,
// enabling stable stacking at lower iteration counts.
const SPECULATIVE_DISTANCE: f32 = 0.05;

fn entity_aabb(e: &dyn PhysicalEntity) -> Aabb {
    let any: &dyn Any = e;

    if let Some(rb) = any.downcast_ref::<RigidBody>() {
        if let Some(col) = &rb.collider {
            let mut aabb = col.aabb(*rb.pos(), rb.angle());
            let ext = Vec2::new(SPECULATIVE_DISTANCE, SPECULATIVE_DISTANCE);
            aabb.min = aabb.min - ext;
            aabb.max = aabb.max + ext;
            return aabb;
        } else {
            let ext = Vec2::new(0.05, 0.05);
            return Aabb::new(*rb.pos() - ext, *rb.pos() + ext);
        }
    }

    if let Some(p) = any.downcast_ref::<Particle>() {
        let col = Collider2D::Circle { radius: 0.05 };
        let mut aabb = col.aabb(*p.pos(), p.angle());
        let ext = Vec2::new(SPECULATIVE_DISTANCE, SPECULATIVE_DISTANCE);
        aabb.min = aabb.min - ext;
        aabb.max = aabb.max + ext;
        return aabb;
    }

    let ext = Vec2::new(0.01, 0.01);
    Aabb::new(*e.pos() - ext, *e.pos() + ext)
}

pub fn detect_sap(entities: &[Box<dyn PhysicalEntity>]) -> Vec<(usize, usize)> {
    struct Entry {
        index: usize,
        aabb: Aabb,
    }

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

    let mut active: Vec<Entry> = Vec::new();
    let mut pairs: Vec<(usize, usize)> = Vec::new();

    for cur in entries {
        active.retain(|e| e.aabb.max.x >= cur.aabb.min.x);
        for e in &active {
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
