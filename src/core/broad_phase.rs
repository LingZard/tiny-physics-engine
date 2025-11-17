use std::any::Any;

use super::entity::PhysicalEntity;
use super::particle::Particle;
use super::rigid_body::RigidBody;
use super::shape::{Aabb, Collider2D};
use crate::math::vec::Vec2;

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
    let ext = Vec2::new(0.01, 0.01);
    let min = e.pos() - &ext;
    let max = e.pos() + &ext;
    Aabb::new(min, max)
}

pub fn broad_phase_sap(entities: &Vec<Box<dyn PhysicalEntity>>) -> Vec<(usize, usize)> {
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
    for cur in entries.into_iter() {
        active.retain(|e| e.aabb.max.x >= cur.aabb.min.x);
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
