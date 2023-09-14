use log::debug;
use nalgebra::{zero, UnitVector3, Vector3};

use crate::state::Agent;

use super::electro::electro_kinematics;

pub fn agent_agent_electro(
    a1: &Agent,
    a2: &Agent,
    agent_radius: f64,
    electro_coeff: f64,
) -> (Vector3<f64>, Vector3<f64>) {
    let might_overlap = a1
        .bounding_box(agent_radius)
        .overlaps(a2.bounding_box(agent_radius));
    if !might_overlap {
        return (zero(), zero());
    }
    let (a1p, a2p) = a1.seg.approx_closest_points_on_segment(&a2.seg, 5);
    let r1c_r2c = a2p - a1p;

    let f = electro_kinematics(
        UnitVector3::new_normalize(-r1c_r2c),
        2.0 * agent_radius - r1c_r2c.magnitude(),
        electro_coeff,
    );

    debug!(
        "ELECTRO a1s={} a1e={} a2s={} a2e={} a1p={} a2p={} r1c_r2c={} f={}",
        1e6 * a1.seg.start.x,
        1e6 * a1.seg.end.x,
        1e6 * a2.seg.start.x,
        1e6 * a2.seg.end.x,
        1e6 * a1p.x,
        1e6 * a2p.x,
        1e6 * r1c_r2c.x,
        1e12 * f.x,
    );

    let r1com_to_r1f = a1p - a1.seg.centroid();

    let torque = r1com_to_r1f.cross(&f);
    (f, torque)
}

// Super naive implementation.
pub fn agent_agents_electro(
    a1: &Agent,
    i1: usize,
    ags: &[Agent],
    agent_radius: f64,
    electro_coeff: f64,
) -> (Vector3<f64>, Vector3<f64>) {
    ags.iter()
        .enumerate()
        .fold((zero(), zero()), |(f_tot, torque_tot), (i2, a2)| {
            if i1 == i2 {
                (f_tot, torque_tot)
            } else {
                let (f, torque) = agent_agent_electro(a1, a2, agent_radius, electro_coeff);
                (f_tot + f, torque_tot + torque)
            }
        })
}
