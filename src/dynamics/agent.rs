use nalgebra::{zero, Point3, UnitVector3, Vector3};

use super::electro::electro_kinematics;

pub fn agent_agent_electro(
    r1: Point3<f64>,
    r2: &Point3<f64>,
    agent_radius: f64,
    electro_coeff: f64,
) -> Vector3<f64> {
    let r1c_r2c = r2 - r1;
    electro_kinematics(
        UnitVector3::new_normalize(r1c_r2c),
        2.0 * agent_radius - r1c_r2c.magnitude(),
        electro_coeff,
    )
}

// Super naive implementation.
pub fn agent_agents_electro(
    i1: usize,
    r1: Point3<f64>,
    rs: &[Point3<f64>],
    agent_radius: f64,
    electro_coeff: f64,
) -> Vector3<f64> {
    rs.iter()
        .enumerate()
        .map(|(i2, r2)| {
            if i1 == i2 {
                zero()
            } else {
                agent_agent_electro(r1, r2, agent_radius, electro_coeff)
            }
        })
        .sum()
}

// Super naive implementation.
pub fn agent_agents_hydro(
    i1: usize,
    r1: Point3<f64>,
    rs: &[Point3<f64>],
    a: Vector3<f64>,
    b: Vector3<f64>,
) -> Vector3<f64> {
    rs.iter()
        .enumerate()
        .map(|(i2, r2)| {
            if i1 == i2 {
                zero()
            } else {
                super::stokes_solutions::stresslet_chwang_u(a, b, r1 - r2)
            }
        })
        .sum()
}
