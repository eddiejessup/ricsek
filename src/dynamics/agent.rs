use nalgebra::{zero, Point3, UnitVector3, Vector3};

use crate::state::Agent;

use super::electro::electro_kinematics;

pub fn agent_x_electro(
    f: Vector3<f64>,
    closest_seg_point: Point3<f64>,
    u_seg_to_obj: UnitVector3<f64>,
    agent_radius: f64,
    a: &Agent,
) -> (Vector3<f64>, Vector3<f64>) {
    // The force is applied at the closest-point on the segment, plus the radius
    // of the agent in the direction of the repulsing object.
    let force_point = closest_seg_point + u_seg_to_obj.scale(agent_radius);
    let moment_arm = force_point - a.seg.centroid();
    let torque = moment_arm.cross(&f);

    (f, torque)
}

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

    agent_x_electro(
        f,
        a1p,
        UnitVector3::new_normalize(r1c_r2c),
        agent_radius,
        a1,
    )
}

// Super naive implementation.
pub fn agent_agents_electro(
    a1: &Agent,
    i1: usize,
    agents: &[Agent],
    agent_radius: f64,
    electro_coeff: f64,
) -> (Vector3<f64>, Vector3<f64>) {
    agents
        .iter()
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

// Super naive implementation.
pub fn agents_agents_electro(
    agents: &[Agent],
    agent_radius: f64,
    electro_coeff: f64,
) -> Vec<(Vector3<f64>, Vector3<f64>)> {
    agents
        .iter()
        .enumerate()
        .map(|(i_agent, agent)| {
            agent_agents_electro(agent, i_agent, &agents, agent_radius, electro_coeff)
        })
        .collect()
}
