use super::Aabb;
use crate::core::body::PhysicalEntity;
use crate::core::params::SimParams;
use crate::math::vec::Vec2;

fn entity_aabb(e: &dyn PhysicalEntity, params: SimParams) -> Aabb {
    if let Some(col) = e.collider() {
        let mut aabb = col.aabb(*e.pos(), e.angle());
        let ext = Vec2::new(params.speculative_distance, params.speculative_distance);
        aabb.min = aabb.min - ext;
        aabb.max = aabb.max + ext;
        return aabb;
    }

    let ext = Vec2::new(0.01, 0.01);
    Aabb::new(*e.pos() - ext, *e.pos() + ext)
}

pub fn detect_sap(entities: &[Box<dyn PhysicalEntity>], params: SimParams) -> Vec<(usize, usize)> {
    struct Entry {
        index: usize,
        aabb: Aabb,
    }

    let mut entries: Vec<Entry> = entities
        .iter()
        .enumerate()
        .map(|(i, e)| Entry {
            index: i,
            aabb: entity_aabb(&**e, params),
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
