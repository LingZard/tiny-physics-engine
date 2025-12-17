use super::manifold::Manifold;
use super::{Collider2D, box_box, box_circle, circle_circle};
use crate::core::body::PhysicalEntity;
use crate::core::params::SimParams;

fn build_manifold_for_pair(
    index_a: usize,
    index_b: usize,
    entity_a: &dyn PhysicalEntity,
    entity_b: &dyn PhysicalEntity,
    params: SimParams,
) -> Option<Manifold> {
    let collider_a = entity_a.collider()?;
    let collider_b = entity_b.collider()?;
    let angle_a = entity_a.angle();
    let angle_b = entity_b.angle();

    let (normal, contacts) = match (collider_a, collider_b) {
        (Collider2D::Circle { radius: ra }, Collider2D::Circle { radius: rb }) => {
            let (n, c) = circle_circle::detect(
                *entity_a.pos(),
                *ra,
                *entity_b.pos(),
                *rb,
                params.speculative_distance,
            )?;
            (n, vec![c])
        }
        (Collider2D::Box { half_extents }, Collider2D::Circle { radius }) => {
            let (n, c) = box_circle::detect(
                *entity_a.pos(),
                angle_a,
                *half_extents,
                *entity_b.pos(),
                *radius,
                params.speculative_distance,
            )?;
            (n, vec![c])
        }
        (Collider2D::Circle { radius }, Collider2D::Box { half_extents }) => {
            let (n, cp) = box_circle::detect(
                *entity_b.pos(),
                angle_b,
                *half_extents,
                *entity_a.pos(),
                *radius,
                params.speculative_distance,
            )?;
            (-n, vec![cp])
        }
        (Collider2D::Box { half_extents: hea }, Collider2D::Box { half_extents: heb }) => {
            box_box::detect(
                *entity_a.pos(),
                angle_a,
                *hea,
                *entity_b.pos(),
                angle_b,
                *heb,
                params.speculative_distance,
            )?
        }
    };

    Some(Manifold::new(index_a, index_b, normal, contacts))
}

pub fn detect(
    entities: &[Box<dyn PhysicalEntity>],
    pairs: &[(usize, usize)],
    params: SimParams,
) -> Vec<Manifold> {
    pairs
        .iter()
        .filter_map(|&(idx_a, idx_b)| {
            let entity_a = entities.get(idx_a)?;
            let entity_b = entities.get(idx_b)?;
            build_manifold_for_pair(idx_a, idx_b, &**entity_a, &**entity_b, params)
        })
        .collect()
}
