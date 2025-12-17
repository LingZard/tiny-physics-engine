use super::manifold::ContactPoint;
use crate::math::mat::Mat2;
use crate::math::vec::Vec2;

const SPECULATIVE_DISTANCE: f32 = 0.05;

fn clip_segment_to_line(v_in: &[Vec2], normal: Vec2, offset: f32) -> Vec<Vec2> {
    let mut v_out = Vec::new();
    if v_in.len() < 2 {
        return v_out;
    }

    let d0 = normal.dot(v_in[0]) - offset;
    let d1 = normal.dot(v_in[1]) - offset;

    if d0 <= 0.0 && d1 <= 0.0 {
        v_out.push(v_in[0]);
        v_out.push(v_in[1]);
    } else if d0 > 0.0 && d1 > 0.0 {
        // both outside
    } else {
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

fn compute_incident_edge(center: Vec2, rot: &Mat2, half: Vec2, ref_normal: Vec2) -> [Vec2; 2] {
    let inv_rot = rot.transpose();
    let local_n = inv_rot.mul_vec2(ref_normal);

    let (v1_local, v2_local) = if local_n.x.abs() > local_n.y.abs() {
        if local_n.x > 0.0 {
            (Vec2::new(-half.x, half.y), Vec2::new(-half.x, -half.y))
        } else {
            (Vec2::new(half.x, -half.y), Vec2::new(half.x, half.y))
        }
    } else {
        if local_n.y > 0.0 {
            (Vec2::new(half.x, -half.y), Vec2::new(-half.x, -half.y))
        } else {
            (Vec2::new(-half.x, half.y), Vec2::new(half.x, half.y))
        }
    };

    [
        rot.mul_vec2(v1_local) + center,
        rot.mul_vec2(v2_local) + center,
    ]
}

pub fn detect(
    center_a: Vec2,
    angle_a: f32,
    half_a: Vec2,
    center_b: Vec2,
    angle_b: f32,
    half_b: Vec2,
) -> Option<(Vec2, Vec<ContactPoint>)> {
    let rot_a = Mat2::rotation(angle_a);
    let rot_b = Mat2::rotation(angle_b);
    let rot_a_t = rot_a.transpose();
    let rot_b_t = rot_b.transpose();

    let dp = center_b - center_a;
    let dp_a = rot_a_t.mul_vec2(dp);
    let dp_b = rot_b_t.mul_vec2(dp);

    let c_ab = rot_a_t.mul_mat2(&rot_b);
    let abs_c = Mat2::new(
        c_ab.m00.abs(),
        c_ab.m01.abs(),
        c_ab.m10.abs(),
        c_ab.m11.abs(),
    );
    let abs_c_t = abs_c.transpose();

    let face_a_x = dp_a.x.abs() - (half_a.x + abs_c.m00 * half_b.x + abs_c.m01 * half_b.y);
    if face_a_x > SPECULATIVE_DISTANCE {
        return None;
    }

    let face_a_y = dp_a.y.abs() - (half_a.y + abs_c.m10 * half_b.x + abs_c.m11 * half_b.y);
    if face_a_y > SPECULATIVE_DISTANCE {
        return None;
    }

    let face_b_x = dp_b.x.abs() - (half_b.x + abs_c_t.m00 * half_a.x + abs_c_t.m01 * half_a.y);
    if face_b_x > SPECULATIVE_DISTANCE {
        return None;
    }

    let face_b_y = dp_b.y.abs() - (half_b.y + abs_c_t.m10 * half_a.x + abs_c_t.m11 * half_a.y);
    if face_b_y > SPECULATIVE_DISTANCE {
        return None;
    }

    let (best_axis, _) = [(0, face_a_x), (1, face_a_y), (2, face_b_x), (3, face_b_y)]
        .iter()
        .fold((0, f32::NEG_INFINITY), |(idx, max), &(i, sep)| {
            if sep > max { (i, sep) } else { (idx, max) }
        });

    let (ref_idx, ref_normal, inc_half, inc_center, inc_rot) = if best_axis < 2 {
        let normal = if best_axis == 0 {
            if dp_a.x > 0.0 {
                Vec2::new(rot_a.m00, rot_a.m10)
            } else {
                Vec2::new(-rot_a.m00, -rot_a.m10)
            }
        } else {
            if dp_a.y > 0.0 {
                Vec2::new(rot_a.m01, rot_a.m11)
            } else {
                Vec2::new(-rot_a.m01, -rot_a.m11)
            }
        };
        (0, normal, half_b, center_b, &rot_b)
    } else {
        let normal = if best_axis == 2 {
            if dp_b.x > 0.0 {
                Vec2::new(-rot_b.m00, -rot_b.m10)
            } else {
                Vec2::new(rot_b.m00, rot_b.m10)
            }
        } else {
            if dp_b.y > 0.0 {
                Vec2::new(-rot_b.m01, -rot_b.m11)
            } else {
                Vec2::new(rot_b.m01, rot_b.m11)
            }
        };
        (1, normal, half_a, center_a, &rot_a)
    };

    let incident_edge = compute_incident_edge(inc_center, inc_rot, inc_half, ref_normal);

    let (ref_center, ref_rot, ref_h) = if ref_idx == 0 {
        (center_a, &rot_a, half_a)
    } else {
        (center_b, &rot_b, half_b)
    };
    let ref_rot_t = ref_rot.transpose();

    let incident_local = [
        ref_rot_t.mul_vec2(incident_edge[0] - ref_center),
        ref_rot_t.mul_vec2(incident_edge[1] - ref_center),
    ];
    let ref_normal_local = ref_rot_t.mul_vec2(ref_normal);

    let (side_n1, off1, side_n2, off2, front_off) =
        if ref_normal_local.x.abs() > ref_normal_local.y.abs() {
            (
                Vec2::new(0.0, 1.0),
                ref_h.y,
                Vec2::new(0.0, -1.0),
                ref_h.y,
                ref_h.x,
            )
        } else {
            (
                Vec2::new(1.0, 0.0),
                ref_h.x,
                Vec2::new(-1.0, 0.0),
                ref_h.x,
                ref_h.y,
            )
        };

    let clip1 = clip_segment_to_line(&incident_local, side_n1, off1);
    if clip1.len() < 2 {
        return None;
    }

    let clip2 = clip_segment_to_line(&clip1, side_n2, off2);
    if clip2.len() < 2 {
        return None;
    }

    let mut contacts = Vec::new();
    for v_local in clip2 {
        let sep = ref_normal_local.dot(v_local) - front_off;
        // Allow small separation for speculative contacts.
        if sep <= SPECULATIVE_DISTANCE {
            contacts.push(ContactPoint {
                point: ref_rot.mul_vec2(v_local) + ref_center,
                // sep>0 => separated (speculative), sep<0 => overlapping
                penetration: -sep,
            });
        }
    }

    if contacts.is_empty() {
        return None;
    }

    let final_normal = if ref_idx == 0 {
        ref_normal
    } else {
        -ref_normal
    };
    Some((final_normal, contacts))
}
