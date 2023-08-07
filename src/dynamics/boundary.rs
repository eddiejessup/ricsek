use nalgebra::{Point3, UnitVector3, Vector3};

use crate::config::setup::parameters::simulation::BoundaryConfig;

use super::electro::electro_kinematics;

fn wrap1(x: f64, l: f64) -> f64 {
    if x < -l * 0.5 {
        // let n_wrap = (x / (l * 0.5)).abs().ceil();
        // if n_wrap > 1.0 {
        //   panic!("n_wrap = {} > 1", n_wrap);
        // }
        x + l
    } else if x > l * 0.5 {
        x - l
    } else {
        x
    }
}

pub fn wrap(r: &mut Point3<f64>, boundaries: &BoundaryConfig) {
    if !boundaries.0.x.closed {
      r.x = wrap1(r.x, boundaries.0.x.l);
    }
    if !boundaries.0.y.closed {
      r.y = wrap1(r.y, boundaries.0.y.l);
    }
    if !boundaries.0.z.closed {
      r.z = wrap1(r.z, boundaries.0.z.l);
    }
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
