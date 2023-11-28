pub mod agent;
pub mod boundary;
pub mod brownian;
pub mod common;
pub mod electro;
pub mod stokes_solutions;

use std::f64::consts::PI;

use crate::config::run::{RunContext, RunParams};
use crate::config::setup::parameters::simulation::SimParams;
use crate::config::setup::parameters::singularities::{Singularity, SingularityParams};
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

pub fn agent_singularities(sim_params: &SimParams, agent: &Agent) -> Vec<Singularity> {
    vec![
        Singularity {
            point: agent.seg.start,
            params: SingularityParams::Stokeslet {
                a: agent
                    .u()
                    .scale(-sim_params.agent_propulsive_stokeslet_strength),
            },
        },
        Singularity {
            point: agent.seg.start,
            params: SingularityParams::Rotlet {
                c: agent
                    .u()
                    .scale(-sim_params.agent_propulsive_rotlet_strength),
            },
        },
        Singularity {
            point: agent.seg.end,
            params: SingularityParams::Stokeslet {
                a: agent
                    .u()
                    .scale(sim_params.agent_propulsive_stokeslet_strength),
            },
        },
        Singularity {
            point: agent.seg.end,
            params: SingularityParams::Rotlet {
                c: agent.u().scale(sim_params.agent_propulsive_rotlet_strength),
            },
        },
    ]
}

pub fn agents_singularities(
    sim_params: &SimParams,
    agents: &[Agent],
) -> Vec<(ObjectPoint, SingularityParams)> {
    agents
        .iter()
        .enumerate()
        .flat_map(|(i_agent, agent)| {
            agent_singularities(sim_params, agent)
                .iter()
                .map(|singularity| {
                    (
                        ObjectPoint {
                            object_id: i_agent as u32,
                            position_com: agent.seg.centroid(),
                            position: singularity.point,
                        },
                        singularity.params.clone(),
                    )
                })
                .collect::<Vec<_>>()
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
        let agent_singularities = agents_singularities(sim_params, &sim_state.agents);

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

        // let fluid_flow = vec![zero(); run_context.sample_eval_points.len()];
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
