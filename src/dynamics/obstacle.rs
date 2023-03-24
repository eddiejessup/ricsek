use crate::{math::capsule::Capsule, state::Agent};

fn agent_obstacle_electro_kinematics(
    y_unit: geo::Point,
    overlap: f64,
    electro_coeff: f64,
) -> (geo::Point, f64) {
    // If agent is further away from the surface than the radius of the agent,
    // then no force.
    // Negative means away from y_unit, meaning repulsion.
    let v_electro_mag = -electro_coeff * overlap.powi(3).sqrt();

    (y_unit * v_electro_mag, 0.0)
}

fn agent_obstacle_hydro_kinematics(
    y_unit: geo::Point,
    y_mag: f64,
    u: geo::Point,
    aspect_ratio: f64,
    hydro_coeff: f64,
) -> (geo::Point, f64) {
    // https://arxiv.org/pdf/0806.2898.pdf
    // The velocity component of the swimmer towards the cap:
    //   v_y(θ, y) = (−3p / 64πηy^2) * (1 − 3 cos^2(θ)).
    // Where:
    // - p is the dipole strength
    // - θ is the angle between the cap normal and the swimmer orientation.
    // - y is the distance between the swimmer and the cap.

    // Angle between the swimmer's direction and the cap surface normal:
    let cos_th = u.dot(y_unit);

    // Repulsion / attraction.
    // Positive means in direction `y_unit`, so the swimmer is attracted to the surface.
    // So this is when (1 - 3 * cos(th)^2) is positive.
    // This is when th > ~54 deg.
    // So the swimmer is attracted when swimming along the surface.
    // When the swimmer is swimming towards the surface, it is repelled.
    let v_hydro_mag = (hydro_coeff / y_mag.powi(2)) * (1.0 - 3.0 * cos_th.powi(2));

    // Rotation.
    let ar_factor = (aspect_ratio.powi(2) - 1.0) / (2.0 * (aspect_ratio.powi(2) + 1.0));
    let sin_th = (1.0 - cos_th.powi(2)).sqrt();
    let om_hydro = -(hydro_coeff * cos_th * sin_th / y_mag.powi(3))
        * (1.0 + ar_factor * (1.0 + cos_th.powi(2)));

    (y_unit * v_hydro_mag, om_hydro)
}

pub fn agent_obstacle_kinematics(
    agent: &Agent,
    capsule: &Capsule,
    agent_radius: f64,
    aspect_ratio: f64,
    hydro_coeff: f64,
    electro_coeff: f64,
) -> (geo::Point, f64) {
    // The vector from the swimmer's centre to the surface.
    // rc_s_dist < 0 means the swimmer's centre is beneath the surface.
    let (_s, _rc_s_vec, rc_s_dist, rc_c_unit) = capsule.closest_point(agent.r);

    // Distance from the surface to the swimmer's *surface*
    // If rs_s_dist < 0, then the swimmer's surface is below the capsule surface.
    let rs_s_dist = rc_s_dist - agent_radius;

    // Electrostatics is only relevant if the swimmer's surface is below the
    // capsule surface.
    let (v_el, om_el) = if rs_s_dist < 0.0 {
      // The overlap is the negative of the above signed distance.
      agent_obstacle_electro_kinematics(rc_c_unit, -rs_s_dist, electro_coeff)
    } else {
      (geo::Point::new(0.0, 0.0), 0.0)
    };

    // Hydrodynamics is only relevant if the swimmer's centre is above the
    // capsule surface.
    let (v_hydro, om_hydro) = if rc_s_dist > 0.0 {
      agent_obstacle_hydro_kinematics(rc_c_unit, rc_s_dist, agent.u, aspect_ratio, hydro_coeff)
    } else {
      (geo::Point::new(0.0, 0.0), 0.0)
    };

    // Consider the cases:
    // The swimmer is completely below the surfce, even its centreline:
    //   Only electrostatics applies.
    // The swimmer is partially below the surface, centrerline is still above the surface:
    //   Have both electrostatics and hydrodynamics.
    // The swimmer is completely above the surface:
    //   Only hydrodynamics applies.

    (v_el + v_hydro, om_el + om_hydro)
}

pub fn agent_obstacles_kinematics(
    agent: &Agent,
    capsules: &[Capsule],
    agent_radius: f64,
    aspect_ratio: f64,
    hydro_coeff: f64,
    electro_coeff: f64,
) -> (geo::Point, f64) {
    capsules
        .iter()
        .map(|cap| {
            agent_obstacle_kinematics(
                agent,
                cap,
                agent_radius,
                aspect_ratio,
                hydro_coeff,
                electro_coeff,
            )
        })
        .fold((geo::Point::new(0.0, 0.0), 0.0), |(v, om), (v1, om1)| {
            (v + v1, om + om1)
        })
}

#[cfg(test)]
mod tests {
    // use super::*;

    // #[test]
    // fn electro_distance() {
    //     let agent_radius = 1.0;
    //     let electro_coeff = 1.0;

    //     let y_unit = geo::coord! {x: 1.0, y: 0.0}.into();
    //     for y_mag in [10.0, 1.0] {
    //         let (res_v, res_om) =
    //             agent_obstacle_electro_kinematics(y_unit, y_mag, agent_radius, electro_coeff);

    //         assert_eq!(res_om, 0.0);
    //         assert_eq!(res_v, geo::coord! {x: 0.0, y: 0.0}.into());
    //     }

    //     let (res_v, res_om) =
    //         agent_obstacle_electro_kinematics(y_unit, 0.9, agent_radius, electro_coeff);
    //     assert_eq!(res_om, 0.0);
    //     assert_eq!(res_v, geo::coord! {x: -0.031622776601683784, y: 0.0}.into());

    //     let y_unit = geo::coord! {x: -1.0, y: 0.0}.into();
    //     let (res_v, res_om) =
    //         agent_obstacle_electro_kinematics(y_unit, 0.9, agent_radius, electro_coeff);
    //     assert_eq!(res_om, 0.0);
    //     assert_eq!(res_v, geo::coord! {x: 0.031622776601683784, y: 0.0}.into());
    // }
}