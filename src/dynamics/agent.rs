use nalgebra::{zero, Point2, UnitVector2, Vector2};

use super::common::electro_kinematics;

pub fn agent_agent_electro(
    r1: Point2<f64>,
    r2: &Point2<f64>,
    agent_radius: f64,
    electro_coeff: f64,
) -> Vector2<f64> {
    let r1c_r2c = r2 - r1;
    electro_kinematics(
        UnitVector2::new_normalize(r1c_r2c),
        2.0 * agent_radius - r1c_r2c.magnitude(),
        electro_coeff,
    )
}

// Super naive implementation.
pub fn agent_agents_electro(
    i1: usize,
    r1: Point2<f64>,
    rs: &[Point2<f64>],
    agent_radius: f64,
    electro_coeff: f64,
) -> Vector2<f64> {
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
    r1: Point2<f64>,
    rs: &[Point2<f64>],
    s_diag: f64,
    s_off: f64,
) -> Vector2<f64> {
    rs.iter()
        .enumerate()
        .map(|(i2, r2)| {
            if i1 == i2 {
                zero()
            } else {
                super::stokes_solutions::stresslet_u(s_diag, s_off, r1 - r2)
            }
        })
        .sum()
}
