//! Box-Box collision detection using SAT (Separating Axis Theorem) and edge clipping.
//!
//! This implementation follows the Box2D-Lite approach:
//! 1. SAT to find the axis of minimum penetration
//! 2. Identify reference and incident faces
//! 3. Clip incident face against reference face side planes
//! 4. Keep points behind the reference face front plane

use super::manifold::ContactPoint;
use crate::math::mat::Mat2;
use crate::math::vec::Vec2;

/// Clips a line segment against a half-plane.
///
/// The half-plane is defined as: `normal · x <= offset`
/// Returns the clipped segment vertices.
fn clip_segment_to_line(v_in: &[Vec2], normal: Vec2, offset: f32) -> Vec<Vec2> {
    let mut v_out = Vec::new();
    if v_in.len() < 2 {
        return v_out;
    }

    // Compute signed distances from the line
    let d0 = normal.dot(&v_in[0]) - offset;
    let d1 = normal.dot(&v_in[1]) - offset;

    // Both inside
    if d0 <= 0.0 && d1 <= 0.0 {
        v_out.push(v_in[0]);
        v_out.push(v_in[1]);
    }
    // Both outside - nothing to keep
    else if d0 > 0.0 && d1 > 0.0 {
        // Empty
    }
    // One inside, one outside - clip
    else {
        let t = d0 / (d0 - d1);
        let intersect = v_in[0] + (v_in[1] - v_in[0]) * t;
        if d0 <= 0.0 {
            v_out.push(v_in[0]);
            v_out.push(intersect);
        } else {
            v_out.push(intersect);
            v_out.push(v_in[1]);
        }
    }
    v_out
}

/// Computes the incident edge on a box given a reference normal.
///
/// The incident edge is the edge most anti-parallel to the reference normal.
fn compute_incident_edge(
    center: &Vec2,
    rot: &Mat2,
    half_extents: &Vec2,
    ref_normal: &Vec2,
) -> [Vec2; 2] {
    // Transform reference normal to local space
    let inv_rot = rot.transpose();
    let local_n = inv_rot.mul_vec2(ref_normal);

    // Find the face most anti-parallel to local_n
    let (v1_local, v2_local) = if local_n.x.abs() > local_n.y.abs() {
        if local_n.x > 0.0 {
            // Left face (negative x)
            (
                Vec2::new(-half_extents.x, half_extents.y),
                Vec2::new(-half_extents.x, -half_extents.y),
            )
        } else {
            // Right face (positive x)
            (
                Vec2::new(half_extents.x, -half_extents.y),
                Vec2::new(half_extents.x, half_extents.y),
            )
        }
    } else {
        if local_n.y > 0.0 {
            // Bottom face (negative y)
            (
                Vec2::new(half_extents.x, -half_extents.y),
                Vec2::new(-half_extents.x, -half_extents.y),
            )
        } else {
            // Top face (positive y)
            (
                Vec2::new(-half_extents.x, half_extents.y),
                Vec2::new(half_extents.x, half_extents.y),
            )
        }
    };

    // Transform to world space
    [
        rot.mul_vec2(&v1_local) + center,
        rot.mul_vec2(&v2_local) + center,
    ]
}

/// Detects collision between two boxes using SAT and edge clipping.
///
/// Returns the collision normal (pointing from A to B) and contact points if colliding.
pub fn detect(
    center_a: &Vec2,
    angle_a: f32,
    half_extents_a: &Vec2,
    center_b: &Vec2,
    angle_b: f32,
    half_extents_b: &Vec2,
) -> Option<(Vec2, Vec<ContactPoint>)> {
    let rot_a = Mat2::rotation(angle_a);
    let rot_b = Mat2::rotation(angle_b);

    let rot_a_t = rot_a.transpose();
    let rot_b_t = rot_b.transpose();

    // Vector from A to B
    let dp = center_b - center_a;
    let dp_a = rot_a_t.mul_vec2(&dp); // dp in A's local space
    let dp_b = rot_b_t.mul_vec2(&dp); // dp in B's local space

    // Rotation matrix from B to A
    let c_ab = rot_a_t.mul_mat2(&rot_b);
    let abs_c = Mat2::new(
        c_ab.m00.abs(),
        c_ab.m01.abs(),
        c_ab.m10.abs(),
        c_ab.m11.abs(),
    );
    let abs_c_t = abs_c.transpose();

    // ========== SAT: Check all 4 axes ==========

    // Box A's X axis
    let face_a_x = dp_a.x.abs()
        - (half_extents_a.x + abs_c.m00 * half_extents_b.x + abs_c.m01 * half_extents_b.y);
    if face_a_x > 0.0 {
        return None;
    }

    // Box A's Y axis
    let face_a_y = dp_a.y.abs()
        - (half_extents_a.y + abs_c.m10 * half_extents_b.x + abs_c.m11 * half_extents_b.y);
    if face_a_y > 0.0 {
        return None;
    }

    // Box B's X axis
    let face_b_x = dp_b.x.abs()
        - (half_extents_b.x + abs_c_t.m00 * half_extents_a.x + abs_c_t.m01 * half_extents_a.y);
    if face_b_x > 0.0 {
        return None;
    }

    // Box B's Y axis
    let face_b_y = dp_b.y.abs()
        - (half_extents_b.y + abs_c_t.m10 * half_extents_a.x + abs_c_t.m11 * half_extents_a.y);
    if face_b_y > 0.0 {
        return None;
    }

    // ========== Find axis of minimum penetration ==========
    let axis_check = [(0, face_a_x), (1, face_a_y), (2, face_b_x), (3, face_b_y)];

    let (best_axis, _) =
        axis_check
            .iter()
            .fold((0, -f32::INFINITY), |(idx, max_sep), &(curr_idx, sep)| {
                if sep > max_sep {
                    (curr_idx, sep)
                } else {
                    (idx, max_sep)
                }
            });

    // ========== Determine reference and incident faces ==========
    let (ref_poly_idx, ref_normal, incident_box_h, incident_box_pos, incident_box_rot) =
        if best_axis < 2 {
            // Reference face is on box A
            let normal = if best_axis == 0 {
                // A's X axis
                if dp_a.x > 0.0 {
                    Vec2::new(rot_a.m00, rot_a.m10)
                } else {
                    Vec2::new(-rot_a.m00, -rot_a.m10)
                }
            } else {
                // A's Y axis
                if dp_a.y > 0.0 {
                    Vec2::new(rot_a.m01, rot_a.m11)
                } else {
                    Vec2::new(-rot_a.m01, -rot_a.m11)
                }
            };
            (0, normal, half_extents_b, center_b, &rot_b)
        } else {
            // Reference face is on box B
            let normal = if best_axis == 2 {
                // B's X axis
                if dp_b.x > 0.0 {
                    Vec2::new(rot_b.m00, rot_b.m10)
                } else {
                    Vec2::new(-rot_b.m00, -rot_b.m10)
                }
            } else {
                // B's Y axis
                if dp_b.y > 0.0 {
                    Vec2::new(rot_b.m01, rot_b.m11)
                } else {
                    Vec2::new(-rot_b.m01, -rot_b.m11)
                }
            };
            (1, normal, half_extents_a, center_a, &rot_a)
        };

    // ========== Compute incident edge ==========
    let incident_edge = compute_incident_edge(
        incident_box_pos,
        incident_box_rot,
        incident_box_h,
        &ref_normal,
    );

    // ========== Get reference box properties ==========
    let (ref_center, ref_rot, ref_h) = if ref_poly_idx == 0 {
        (center_a, &rot_a, half_extents_a)
    } else {
        (center_b, &rot_b, half_extents_b)
    };
    let ref_rot_t = ref_rot.transpose();

    // ========== Transform incident edge to reference local space ==========
    let incident_edge_local = [
        ref_rot_t.mul_vec2(&(incident_edge[0] - ref_center)),
        ref_rot_t.mul_vec2(&(incident_edge[1] - ref_center)),
    ];

    let ref_normal_local = ref_rot_t.mul_vec2(&ref_normal);

    // ========== Clip against side planes ==========
    let (side_n1, offset1, side_n2, offset2, front_offset) =
        if ref_normal_local.x.abs() > ref_normal_local.y.abs() {
            // Reference face is perpendicular to X axis
            (
                Vec2::new(0.0, 1.0), // Clip against +Y side
                ref_h.y,
                Vec2::new(0.0, -1.0), // Clip against -Y side
                ref_h.y,
                ref_h.x, // Front plane offset
            )
        } else {
            // Reference face is perpendicular to Y axis
            (
                Vec2::new(1.0, 0.0), // Clip against +X side
                ref_h.x,
                Vec2::new(-1.0, 0.0), // Clip against -X side
                ref_h.x,
                ref_h.y, // Front plane offset
            )
        };

    // First clip
    let clip1 = clip_segment_to_line(&incident_edge_local, side_n1, offset1);
    if clip1.len() < 2 {
        return None;
    }

    // Second clip
    let clip2 = clip_segment_to_line(&clip1, side_n2, offset2);
    if clip2.len() < 2 {
        return None;
    }

    // ========== Keep points behind reference face ==========
    let mut contacts = Vec::new();
    for v_local in clip2 {
        let separation = ref_normal_local.dot(&v_local) - front_offset;
        if separation <= 0.0 {
            let v_world = ref_rot.mul_vec2(&v_local) + ref_center;
            contacts.push(ContactPoint {
                point: v_world,
                penetration: -separation,
            });
        }
    }

    if contacts.is_empty() {
        return None;
    }

    // Normal should point from A to B
    let final_normal = if ref_poly_idx == 0 {
        ref_normal
    } else {
        -ref_normal
    };

    Some((final_normal, contacts))
}
