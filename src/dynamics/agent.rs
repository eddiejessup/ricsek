use nalgebra::{Point3, UnitVector3, Vector3};
use num_traits::Zero;

use crate::config::setup::parameters::{
    common::BoundaryConfig, simulation::SimParams, singularities::SingularityParams,
};
use crate::geometry::{
    capsule::capsule_bounding_box, line_segment::LineSegment, point::ObjectPoint,
};
use crate::state::*;

use super::fluid::get_singularity_image;
use super::{common::Wrench, electro::electro_kinematics};

pub fn capsule_electro_torque(
    f: Vector3<f64>,
    closest_seg_point: Point3<f64>,
    u_seg_to_obj: UnitVector3<f64>,
    radius: f64,
    s: &LineSegment,
) -> Vector3<f64> {
    // The force is applied at the closest-point on the segment, plus the radius
    // of the capsule in the direction of the repulsing object.
    let force_point = closest_seg_point + u_seg_to_obj.scale(radius);
    let moment_arm = force_point - s.centroid();
    // println!("force_point={}, arm={}", force_point, moment_arm);
    moment_arm.cross(&f)
}

pub fn capsule_capsule_electro(
    seg1: &LineSegment,
    seg2: &LineSegment,
    radius: f64,
    electro_coeff: f64,
) -> Wrench {
    let might_overlap =
        capsule_bounding_box(seg1, radius).overlaps(capsule_bounding_box(seg2, radius));
    if !might_overlap {
        return Wrench::zero();
    }
    let (seg1p, seg2p) = seg1.approx_closest_points_on_segment(seg2, 5);
    let r1c_r2c = seg2p - seg1p;

    let force = electro_kinematics(
        UnitVector3::new_normalize(-r1c_r2c),
        2.0 * radius - r1c_r2c.magnitude(),
        electro_coeff,
    );

    let torque = capsule_electro_torque(
        force,
        seg1p,
        UnitVector3::new_normalize(r1c_r2c),
        radius,
        seg1,
    );
    println!(
        "closest-point-on-i={}, overlap={}, force={}, torque={}",
        seg1p,
        2.0 * radius - r1c_r2c.magnitude(),
        force,
        torque
    );
    Wrench { force, torque }
}

// Super naive implementation.
pub fn capsule_capsules_electro(
    seg1: &LineSegment,
    i1: usize,
    segs: &[LineSegment],
    radius: f64,
    electro_coeff: f64,
) -> Wrench {
    segs.iter()
        .enumerate()
        .fold(Wrench::zero(), |w_tot, (i2, seg2)| {
            if i1 == i2 {
                w_tot
            } else {
                w_tot + capsule_capsule_electro(seg1, seg2, radius, electro_coeff)
            }
        })
}

// Super naive implementation.
pub fn capsules_capsules_electro(
    segs: &[LineSegment],
    radius: f64,
    electro_coeff: f64,
) -> Vec<Wrench> {
    segs.iter()
        .enumerate()
        .map(|(i_capsule, capsule)| {
            capsule_capsules_electro(capsule, i_capsule, segs, radius, electro_coeff)
        })
        .collect()
}

pub fn agent_singularities(
    sim_params: &SimParams,
    agent: &Agent,
    i_agent: u32,
    boundaries: &BoundaryConfig,
) -> Vec<(ObjectPoint, SingularityParams)> {
    let base_singularities = vec![
        (
            ObjectPoint {
                object_id: i_agent,
                position_com: agent.seg.centroid(),
                position: agent.seg.start,
            },
            SingularityParams::Stokeslet {
                a: agent
                    .u()
                    .scale(-sim_params.agent_propulsive_stokeslet_strength),
            },
        ),
        // (
        //     ObjectPoint {
        //         object_id: i_agent,
        //         position_com: agent.seg.centroid(),
        //         position: agent.seg.start,
        //     },
        //     SingularityParams::Rotlet {
        //         c: agent
        //             .u()
        //             .scale(-sim_params.agent_propulsive_rotlet_strength),
        //     },
        // ),
        (
            ObjectPoint {
                object_id: i_agent,
                position_com: agent.seg.centroid(),
                position: agent.seg.end,
            },
            SingularityParams::Stokeslet {
                a: agent
                    .u()
                    .scale(sim_params.agent_propulsive_stokeslet_strength),
            },
        ),
        // (
        //     ObjectPoint {
        //         object_id: i_agent,
        //         position_com: agent.seg.centroid(),
        //         position: agent.seg.end,
        //     },
        //     SingularityParams::Rotlet {
        //         c: agent.u().scale(sim_params.agent_propulsive_rotlet_strength),
        //     },
        // ),
    ];

    let image_singularities: Vec<_> = base_singularities
        .iter()
        .flat_map(|(p, params)| get_singularity_image(p, params, boundaries))
        .collect();

    // Return the concatenation of base and image.
    base_singularities
        .into_iter()
        .chain(image_singularities)
        .collect()
}

pub fn agents_singularities(
    sim_params: &SimParams,
    agents: &[Agent],
    boundaries: &BoundaryConfig,
) -> Vec<(ObjectPoint, SingularityParams)> {
    agents
        .iter()
        .enumerate()
        .flat_map(|(i_agent, agent)| {
            agent_singularities(sim_params, agent, i_agent as u32, boundaries)
        })
        .collect()
}
