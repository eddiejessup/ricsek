use crate::math::point;

use super::common::electro_kinematics;

pub fn agent_agent_electro(
    r1: &geo::Point,
    r2: &geo::Point,
    agent_radius: f64,
    electro_coeff: f64,
) -> geo::Point {
    let r1c_r2c = *r2 - *r1;
    let r1c_r2c_dist = point::point_magnitude(r1c_r2c);
    electro_kinematics(
        r1c_r2c / r1c_r2c_dist,
        2.0 * agent_radius - r1c_r2c_dist,
        electro_coeff,
    )
}

// Super naive implementation.
pub fn agent_agents_electro(
    i1: usize,
    r1: &geo::Point,
    rs: &[geo::Point],
    agent_radius: f64,
    electro_coeff: f64,
) -> geo::Point {
    rs.iter()
        .enumerate()
        .map(|(i2, r2)| {
            if i1 == i2 {
                geo::Point::new(0.0, 0.0)
            } else {
                agent_agent_electro(r1, r2, agent_radius, electro_coeff)
            }
        })
        .fold(geo::Point::new(0.0, 0.0), |r1, r2| r1 + r2)
}
