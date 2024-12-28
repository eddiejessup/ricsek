use nalgebra::{Point3, Vector3};
use num_traits::Zero;

use crate::{
    config::setup::parameters::common::{AxisBoundaryConfig, BoundaryConfig},
    geometry::reverse_unit_vector,
    state::Agent,
};

use super::{agent::capsule_electro_torque, common::Wrench, electro::electro_kinematics};

fn wrap1(com: f64, x: f64, l: f64) -> f64 {
    match (com / (l * 0.5)) as i64 {
        0 => x,
        1 => x - l,
        -1 => x + l,
        2 => x - 2.0 * l,
        -2 => x + 2.0 * l,
        n => panic!("Unexpected n_wrap: {} (com={}, x={}, l={})", n, com, x, l),
    }
}

fn wrap1c(com: f64, x: f64, boundary: &AxisBoundaryConfig) -> f64 {
    if boundary.closed {
        x
    } else {
        wrap1(com, x, boundary.l)
    }
}

pub fn wrap_inplace(com: Point3<f64>, r: &mut Point3<f64>, boundaries: &BoundaryConfig) {
    r.x = wrap1c(com.x, r.x, &boundaries.0.x);
    r.y = wrap1c(com.y, r.y, &boundaries.0.y);
    r.z = wrap1c(com.z, r.z, &boundaries.0.z);
}

pub fn wrap_vec_inplace(com: Vector3<f64>, r: &mut Vector3<f64>, boundaries: &BoundaryConfig) {
    r.x = wrap1c(com.x, r.x, &boundaries.0.x);
    r.y = wrap1c(com.y, r.y, &boundaries.0.y);
    r.z = wrap1c(com.z, r.z, &boundaries.0.z);
}

pub fn wrap_agent_inplace(a: &mut Agent, boundaries: &BoundaryConfig) {
    let com = a.seg.centroid();
    wrap_inplace(com, &mut a.seg.start, boundaries);
    wrap_inplace(com, &mut a.seg.end, boundaries);
}

pub fn wrap_vec(com: Vector3<f64>, r: Vector3<f64>, boundaries: &BoundaryConfig) -> Vector3<f64> {
    Vector3::new(
        wrap1c(com.x, r.x, &boundaries.0.x),
        wrap1c(com.y, r.y, &boundaries.0.y),
        wrap1c(com.z, r.z, &boundaries.0.z),
    )
}

pub fn wrap(com: Point3<f64>, r: Point3<f64>, boundaries: &BoundaryConfig) -> Point3<f64> {
    wrap_vec(com.coords, r.coords, boundaries).into()
}

pub fn minimum_image_vector(
    p_to: Point3<f64>,
    p_to_com: Point3<f64>,
    p_from: Point3<f64>,
    p_from_com: Point3<f64>,
    boundaries: &BoundaryConfig,
) -> Vector3<f64> {
    let r_com: Vector3<f64> = p_to_com - p_from_com;
    let mut r: Vector3<f64> = p_to - p_from;
    wrap_vec_inplace(r_com, &mut r, boundaries);
    r
}

pub fn pairwise_dist(
    r: Point3<f64>,
    rs: &[Point3<f64>],
    boundaries: &BoundaryConfig,
) -> Vec<Vector3<f64>> {
    rs.iter()
        .map(|r2| {
            let mut dr = r2 - r;
            wrap_vec_inplace(dr, &mut dr, boundaries);
            dr
        })
        .collect()
}

pub fn boundary_electro(
    a: &Agent,
    agent_radius: f64,
    boundaries: &BoundaryConfig,
    electro_coeff: f64,
) -> Wrench {
    boundaries
        .agent_closest_points_on_boundaries(a)
        .iter()
        .fold(
            Wrench::zero(),
            |w_tot, (closest_point_segment, normal, seg_overlap)| {
                let overlap = agent_radius + seg_overlap;
                let force = electro_kinematics(*normal, overlap, electro_coeff);

                let torque = capsule_electro_torque(
                    force,
                    *closest_point_segment,
                    reverse_unit_vector(normal),
                    agent_radius,
                    &a.seg,
                );

                w_tot + Wrench { force, torque }
            },
        )
}
