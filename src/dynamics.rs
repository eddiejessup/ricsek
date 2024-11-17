pub mod agent;
pub mod boundary;
pub mod brownian;
pub mod common;
pub mod electro;
pub mod stokes_solutions;

use std::f64::consts::PI;

use crate::config::run::{RunContext, RunParams};
use crate::config::setup::parameters::common::BoundaryConfig;
use crate::config::setup::parameters::simulation::SimParams;
use crate::config::setup::parameters::singularities::SingularityParams;
use crate::dynamics::common::zero_wrench;
use crate::geometry::line_segment::LineSegment;
use crate::geometry::point::{random_vector, ObjectPoint};
use crate::state::*;
use diesel::result::Error;
use log::{debug, info};
use nalgebra::{zero, Point3, Rotation3, UnitVector3, Vector3};
use rand_distr::Uniform;

use self::brownian::brownian_distr;

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct AgentStepSummary {
    pub f_agent_electro: Vector3<f64>,
    pub torque_agent_electro: Vector3<f64>,
    pub f_propulsion: Vector3<f64>,
    pub f_boundary_electro: Vector3<f64>,
    pub torque_boundary_electro: Vector3<f64>,
    pub v_fluid: Vector3<f64>,
}

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct StepSummary {
    pub agent_summaries: Vec<AgentStepSummary>,
    pub fluid_flow: Vec<Vector3<f64>>,
}

pub fn get_singularity_image_across_plane(
    d: Vector3<f64>,
    params: &SingularityParams,
) -> Vec<SingularityParams> {
    let h = d.norm();
    if h < 1e-12 {
        panic!("d is too small");
    }

    let d_direction = d.normalize();

    match params {
        SingularityParams::Stokeslet { a: stokeslet_force } => {
            let stokeslet_force_normal = (stokeslet_force.dot(&d) / h.powi(2)) * d;
            let stokeslet_force_tangent = stokeslet_force - stokeslet_force_normal;

            let stokeslet_strength_normal = stokeslet_force_normal.norm();
            let stokeslet_strength_tangent = stokeslet_force_tangent.norm();

            let stokeslet_direction_normal = if stokeslet_strength_normal > 0.0 {
                stokeslet_force_normal.normalize()
            } else {
                zero()
            };
            let stokeslet_direction_tangent = if stokeslet_strength_tangent > 0.0 {
                stokeslet_force_tangent.normalize()
            } else {
                zero()
            };

            let stokes_doublet_normal_mag = (2.0 * h * stokeslet_strength_normal).sqrt();
            let stokes_doublet_tangent_mag = (2.0 * h * stokeslet_strength_tangent).sqrt();
            let potential_doublet_mag = 2.0 * h.powi(2);

            // https://sci-hub.ee/10.1017/S0305004100049902
            vec![
                // Stokeslet
                SingularityParams::Stokeslet {
                    a: -stokeslet_force,
                },
                // Stokes doublet, normal
                SingularityParams::StokesDoublet {
                    a: -stokes_doublet_normal_mag * stokeslet_direction_normal,
                    b: -stokes_doublet_normal_mag * d_direction,
                },
                // Potential doublet, normal
                SingularityParams::PotentialDoublet {
                    d: potential_doublet_mag * stokeslet_force_normal,
                },
                // Stokes doublet, tangential
                SingularityParams::StokesDoublet {
                    a: -stokes_doublet_tangent_mag * stokeslet_direction_tangent,
                    b: stokes_doublet_tangent_mag * d_direction,
                },
                // Potential doublet, tangential
                SingularityParams::PotentialDoublet {
                    d: -potential_doublet_mag * stokeslet_force_tangent,
                },
            ]
        }
        SingularityParams::StokesDoublet { a: _, b: _ } => {
            panic!("StokesDoublet not supported")
        }
        SingularityParams::Rotlet { c: _ } => {
            panic!("Rotlet not supported")
        }
        SingularityParams::Stresslet { a: _, b: _ } => {
            panic!("Stresslet not supported")
        }
        SingularityParams::PotentialDoublet { d: _ } => {
            panic!("PotentialDoublet not supported")
        }
    }
}

pub fn get_singularity_image(
    p: &ObjectPoint,
    params: &SingularityParams,
    boundaries: &BoundaryConfig,
) -> Vec<(ObjectPoint, SingularityParams)> {
    let param_sets_per_axis: Vec<Vec<(ObjectPoint, SingularityParams)>> = boundaries
        .0
        .iter()
        .enumerate()
        .filter_map(|(i_axis, axis)| {
            if axis.closed {
                // Find the vector pointing to the nearest boundary along this axis.
                let d = Vector3::ith(
                    i_axis,
                    if p.position_com[i_axis] > 0.0 {
                        1.0
                    } else {
                        -1.0
                    } * (axis.l_half() - p.position[i_axis]),
                );
                Some(
                    get_singularity_image_across_plane(d, params)
                        .iter()
                        .map(|params| {
                            (
                                // Place each singularity on the opposite side of the boundary.
                                ObjectPoint {
                                    // ID to indicate fictional singularities.
                                    object_id: std::u32::MAX - 1,
                                    position_com: p.position_com + 2.0 * d,
                                    position: p.position + 2.0 * d,
                                },
                                params.clone(),
                            )
                        })
                        .collect(),
                )
            } else {
                None
            }
        })
        .collect();
    if param_sets_per_axis.len() > 1 {
        panic!("More than one boundary is closed");
    }
    param_sets_per_axis.iter().cloned().flatten().collect()
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
        .chain(image_singularities.into_iter())
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

pub fn force_balance_v(
    sim_params: &SimParams,
    v_fluid: Vector3<f64>,
    f: Vector3<f64>,
) -> Vector3<f64> {
    v_fluid + f * sim_params.agent_translational_mobility
}
pub fn force_balance_omega(
    sim_params: &SimParams,
    omega_fluid: Vector3<f64>,
    torque: Vector3<f64>,
) -> Vector3<f64> {
    omega_fluid + torque * sim_params.agent_rotational_mobility
}

pub fn update(
    sim_params: &SimParams,
    sim_state: &mut SimState,
    run_context: &RunContext,
    rng: &mut rand::rngs::ThreadRng,
) -> StepSummary {
    let uniform_theta_dist = Uniform::new(0.0, 2.0 * PI);
    let uniform_cos_phi_dist = Uniform::new(-1.0, 1.0);
    let trans_diff_dist = brownian_distr(
        sim_params.agent_translational_diffusion_coefficient,
        sim_params.dt,
    );
    let rot_diff_dist = brownian_distr(
        sim_params.agent_rotational_diffusion_coefficient,
        sim_params.dt,
    );

    // Agent-agent electrostatic repulsion.
    let agent_agent_electro_wrenches: Vec<_> = if sim_params.enable_agent_agent_electro {
        run_context.gpu_electro_context.evaluate(
            &sim_state
                .agents
                .iter()
                .map(|agent| agent.seg.clone())
                .collect::<Vec<LineSegment>>(),
        )
    } else {
        vec![zero_wrench(); sim_state.agents.len()]
    };

    let (fluid_twists_at_agents, fluid_flow): (Vec<_>, Vec<_>) = if sim_params.enable_fluid {
        let agent_singularities =
            agents_singularities(sim_params, &sim_state.agents, &sim_params.boundaries);

        let agent_eval_points: Vec<_> = sim_state
            .agents
            .iter()
            .enumerate()
            .map(|(i_agent, agent)| ObjectPoint {
                object_id: i_agent as u32,
                position_com: agent.seg.centroid(),
                position: agent.seg.centroid(),
            })
            .collect();
        let fluid_twists_at_agents = run_context
            .gpu_fluid_agent_context
            .evaluate(&agent_eval_points, &agent_singularities);

        let fluid_flow = run_context
            .gpu_fluid_sample_context
            .evaluate(&run_context.sample_eval_points, &agent_singularities)
            .iter()
            .map(|twist| twist.0)
            .collect();

        (fluid_twists_at_agents, fluid_flow)
    } else {
        let fluid_twists_at_agents = vec![(zero(), zero()); sim_state.agents.len()];
        let fluid_flow = vec![zero(); run_context.sample_eval_points.len()];
        (fluid_twists_at_agents, fluid_flow)
    };

    let mut agent_summaries: Vec<AgentStepSummary> = vec![];
    for (i_agent, mut agent) in sim_state.agents.iter_mut().enumerate() {
        // Fluid velocity and rotation.

        // Agent propulsion.
        // We know there's no torque because it acts along the agent's axis.
        debug!("Computing propulsion force");
        let f_propulsion = if sim_params.enable_agent_propulsion {
            agent.u().scale(sim_params.agent_propulsion_force)
        } else {
            zero()
        };

        debug!("Computing electrostatic repulsion between agents and agents");
        let agent_agent_electro_wrench = agent_agent_electro_wrenches[i_agent].clone();

        debug!("Computing electrostatic repulsion between agents and boundaries");
        let agent_boundary_electro_wrench = if sim_params.enable_agent_boundary_electro {
            boundary::boundary_electro(
                agent,
                sim_params.agent_radius,
                &sim_params.boundaries,
                sim_params.agent_object_hertz_force_coefficient,
            )
        } else {
            zero_wrench()
        };

        debug!("Doing force balance calculation for linear velocity");
        let fluid_twist = fluid_twists_at_agents[i_agent];
        let v_object = force_balance_v(
            sim_params,
            fluid_twist.0,
            f_propulsion + agent_agent_electro_wrench.force + agent_boundary_electro_wrench.force,
        );
        let dr_object = v_object * sim_params.dt + random_vector(rng, trans_diff_dist);
        // Compute translational diffusion translation.
        agent.seg.translate(dr_object);

        debug!("Doing force balance calculation for angular velocity");
        let omega_fluid = fluid_twist.1;
        let omega_object = force_balance_omega(
            sim_params,
            omega_fluid,
            agent_agent_electro_wrench.torque + agent_boundary_electro_wrench.torque,
        );
        let dtheta_object = omega_object * sim_params.dt;
        let rot_object = Rotation3::from_axis_angle(
            &UnitVector3::new_normalize(dtheta_object),
            dtheta_object.norm(),
        );

        // Compute rotational diffusion rotation.
        let rot_rotational_diffusion = brownian::random_rotation(
            rng,
            &uniform_theta_dist,
            &uniform_cos_phi_dist,
            &rot_diff_dist,
        );

        agent.seg.rotate(rot_rotational_diffusion * rot_object);

        // Apply periodic boundary condition.
        boundary::wrap_agent_inplace(&mut agent, &sim_params.boundaries);

        agent_summaries.push(AgentStepSummary {
            f_agent_electro: agent_agent_electro_wrench.force,
            torque_agent_electro: agent_agent_electro_wrench.torque,
            f_propulsion,
            torque_boundary_electro: agent_boundary_electro_wrench.torque,
            f_boundary_electro: agent_boundary_electro_wrench.force,
            v_fluid: fluid_twist.0,
        });
    }

    // Upate time and step.
    sim_state.t += sim_params.dt;
    sim_state.step += 1;

    StepSummary {
        agent_summaries,
        fluid_flow,
    }
}

pub fn run_steps(
    sim_params: &SimParams,
    sim_state: &mut SimState,
    run_context: &RunContext,
    rng: &mut rand::rngs::ThreadRng,
    num_steps: usize,
) -> Option<StepSummary> {
    let mut summary = None;
    for _ in 0..num_steps {
        summary = Some(update(&sim_params, sim_state, &run_context, rng));
    }
    summary
}

pub fn run(
    conn: &mut diesel::PgConnection,
    sim_params: SimParams,
    sample_points: Vec<Point3<f64>>,
    mut sim_state: SimState,
    run_params: RunParams,
) -> Result<(), Error> {
    let rng = &mut rand::thread_rng();
    let run_context = RunContext::new(&sim_params, sim_state.agents.len(), sample_points);

    while sim_state.t < run_params.t_max {
        let summary = update(&sim_params, &mut sim_state, &run_context, rng);

        if sim_state.step % run_params.dstep_view == 0 {
            // Print time with resolution of 0.1ms.
            info!(
                "CHECKPOINT: step={:>8}, t = {:>9.4}",
                sim_state.step, sim_state.t
            );
            crate::db::write_checkpoint(conn, run_params.run_id, &sim_state, Some(summary))?;
        }
    }
    Ok(())
}
