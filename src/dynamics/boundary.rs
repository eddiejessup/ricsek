use log::debug;
use nalgebra::{Point3, Vector3};

use crate::{
    config::setup::parameters::common::{AxisBoundaryConfig, BoundaryConfig},
    geometry::closest::Closest,
    state::Agent,
};

use super::electro::electro_kinematics;

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

pub fn wrap(com: Point3<f64>, r: Point3<f64>, boundaries: &BoundaryConfig) -> Point3<f64> {
    Point3::new(
        wrap1c(com.x, r.x, &boundaries.0.x),
        wrap1c(com.y, r.y, &boundaries.0.y),
        wrap1c(com.z, r.z, &boundaries.0.z),
    )
}

pub fn wrap_vec(com: Vector3<f64>, r: Vector3<f64>, boundaries: &BoundaryConfig) -> Vector3<f64> {
    wrap(com.into(), r.into(), boundaries).coords
}

pub fn pairwise_dist(
    r: Point3<f64>,
    rs: &Vec<Point3<f64>>,
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

// pub fn pairwise_agent_dist(
//     a: &Agent,
//     ags: &[Agent],
//     boundaries: &BoundaryConfig,
// ) -> Vec<Vector3<f64>> {
//     ags.iter()
//         .map(|a2| {
//             let (r_close_1, r_close_2) = a.seg.closest_point_on_line_segment(&a2.seg);
//             debug!("r_close_1: {}, r_close_2: {}", 1e6*r_close_1, 1e6*r_close_2);
//             let mut dr = r_close_2 - r_close_1;
//             wrap_vec_inplace(dr, &mut dr, boundaries);
//             dr
//         })
//         .collect()
// }

pub fn boundary_electro<T: Closest>(
    a: &T,
    boundaries: &BoundaryConfig,
    electro_coeff: f64,
) -> Vector3<f64> {
    boundaries.closed_boundary_overlaps(a).iter().fold(
        Vector3::zeros(),
        |f_tot, (normal, overlap)| {
            let f = electro_kinematics(*normal, *overlap, electro_coeff);
            if *overlap > 0.0 {
                debug!(
                    "normal: {}, overlap: {}, f={}",
                    normal.into_inner(),
                    1e6 * overlap,
                    1e12 * f
                );
            }
            f_tot + f
        },
    )
}
