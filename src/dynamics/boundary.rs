use nalgebra::{Point3, UnitVector3, Vector3};

use crate::config::setup::parameters::common::{AxisBoundaryConfig, BoundaryConfig};

use super::electro::electro_kinematics;

fn wrap1(x: f64, l: f64) -> f64 {
    match (x / (l * 0.5)) as i64 {
        0 => x,
        1 => x - l,
        -1 => x + l,
        n => panic!("Unexpected n_wrap: {}", n),
    }
}

fn wrap1c(x: f64, boundary: &AxisBoundaryConfig) -> f64 {
    if boundary.closed {
        x
    } else {
        wrap1(x, boundary.l)
    }
}

pub fn wrap_inplace(r: &mut Point3<f64>, boundaries: &BoundaryConfig) {
    r.x = wrap1c(r.x, &boundaries.0.x);
    r.y = wrap1c(r.y, &boundaries.0.y);
    r.z = wrap1c(r.z, &boundaries.0.z);
}

pub fn wrap_vec_inplace(r: &mut Vector3<f64>, boundaries: &BoundaryConfig) {
    r.x = wrap1c(r.x, &boundaries.0.x);
    r.y = wrap1c(r.y, &boundaries.0.y);
    r.z = wrap1c(r.z, &boundaries.0.z);
}

pub fn wrap(r: Point3<f64>, boundaries: &BoundaryConfig) -> Point3<f64> {
    Point3::new(
        wrap1c(r.x, &boundaries.0.x),
        wrap1c(r.y, &boundaries.0.y),
        wrap1c(r.z, &boundaries.0.z),
    )
}

pub fn wrap_vec(r: Vector3<f64>, boundaries: &BoundaryConfig) -> Vector3<f64> {
    wrap(r.into(), boundaries).coords
}

pub fn pairwise_dist(
    r: Point3<f64>,
    rs: &Vec<Point3<f64>>,
    boundaries: &BoundaryConfig,
) -> Vec<Vector3<f64>> {
    rs.iter()
        .map(|r2| {
            let mut dr = r2 - r;
            wrap_vec_inplace(&mut dr, boundaries);
            dr
        })
        .collect()
}

pub fn agent_boundary_electro(
    r: Point3<f64>,
    boundaries: &BoundaryConfig,
    agent_radius: f64,
    electro_coeff: f64,
) -> Vector3<f64> {
    // Go through each axis
    // For each axis, find the nearest point on the boundary,
    // Which is at:
    // - The same position as 'r' for all non-axis components
    // - +-l/2 along the axis, depending on the sign of r[i]
    // Then compute the force due to that boundary,
    // and accumulate it for each closed axis.
    let mut f_tot = Vector3::zeros();
    for i in 0..3 {
        if !boundaries.0[i].closed {
            continue;
        }
        let mut nearest_axis_boundary = r.clone();
        nearest_axis_boundary[i] = boundaries.0[i].l / 2.0 * if r[i] > 0.0 { 1.0 } else { -1.0 };
        let rc_boundary = nearest_axis_boundary - r;
        f_tot += electro_kinematics(
            UnitVector3::new_normalize(rc_boundary),
            agent_radius - rc_boundary.magnitude(),
            electro_coeff,
        );
    }
    f_tot
}
