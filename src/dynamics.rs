pub mod agent;
pub mod boundary;
pub mod brownian;
pub mod common;
pub mod electro;
pub mod stokes_solutions;

use std::f64::consts::PI;

use crate::config::run::RunConfig;
use crate::config::setup::parameters::simulation::SimParams;
use crate::config::setup::parameters::singularities::{Singularity, SingularityParams};
use crate::dynamics::common::zero_wrench;
use crate::geometry::point::random_vector;
use crate::{cuda, state::*};
use diesel::result::Error;
use log::{debug, info};
use nalgebra::{zero, Rotation3, UnitVector3, Vector3};
use rand_distr::{Normal, Uniform};

use self::brownian::brownian_distr;

pub fn agents_fluid_twists(
    sim_params: &SimParams,
    agents: &[Agent],
    gpu_fluid_context: &cuda::fluid::CudaFluidContext,
) -> Vec<(Vector3<f64>, Vector3<f64>)> {
    let eval_points: Vec<_> = agents
        .iter()
        .enumerate()
        .map(|(i_agent, agent)| (i_agent as u32, agent.seg.centroid()))
        .collect();
    let singularities: Vec<_> = agents
        .iter()
        .enumerate()
        .flat_map(|(i_agent, agent)| {
            vec![
                (
                    i_agent as u32,
                    Singularity {
                        point: agent.seg.start,
                        params: SingularityParams::Stokeslet {
                            a: agent
                                .u()
                                .scale(-sim_params.agent_propulsive_stokeslet_strength),
                        },
                    },
                ),
                (
                    i_agent as u32,
                    Singularity {
                        point: agent.seg.end,
                        params: SingularityParams::Stokeslet {
                            a: agent
                                .u()
                                .scale(sim_params.agent_propulsive_stokeslet_strength),
                        },
                    },
                ),
            ]
        })
        .collect();
    gpu_fluid_context.evaluate(&eval_points, &singularities)
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
    trans_diff_distr: Normal<f64>,
    rot_diff_distr: Normal<f64>,
    gpu_fluid_context: &cuda::fluid::CudaFluidContext,
    gpu_electro_context: &cuda::electro::CudaElectroContext,
    rng: &mut rand::rngs::ThreadRng,
) -> Vec<AgentStepSummary> {
    let enable_fluid = true;
    let enable_agent_electro = true;
    let enable_boundary_electro = true;
    let enable_propulsion = true;

    let uniform_theta = Uniform::new(0.0, 2.0 * PI);
    let uniform_cos_phi = Uniform::new(-1.0, 1.0);

    // Agent-agent electrostatic repulsion.
    let agent_segments: Vec<_> = sim_state
        .agents
        .iter()
        .map(|agent| agent.seg.clone())
        .collect();
    // let agent_agent_electro_wrenches: Vec<_> = capsules_capsules_electro(
    //     &agent_segments,
    //     sim_params.agent_radius,
    //     sim_params.agent_object_hertz_force_coefficient,
    // );
    let agent_agent_electro_wrenches: Vec<_> = if enable_agent_electro {
        gpu_electro_context.evaluate(&agent_segments)
    } else {
        vec![zero_wrench(); sim_state.agents.len()]
    };

    let fluid_twists: Vec<_> = if enable_fluid {
        agents_fluid_twists(sim_params, &sim_state.agents, gpu_fluid_context)
    } else {
        vec![(zero(), zero()); sim_state.agents.len()]
    };

    let mut agent_summaries: Vec<AgentStepSummary> = vec![];
    for (i_agent, mut agent) in sim_state.agents.iter_mut().enumerate() {
        // Fluid velocity and rotation.

        // Agent propulsion.
        // We know there's no torque because it acts along the agent's axis.
        debug!("Computing propulsion force");
        let f_propulsion = if enable_propulsion {
            agent.u().scale(sim_params.agent_propulsion_force)
        } else {
            zero()
        };

        debug!("Computing electrostatic repulsion between agents and agents");
        let agent_agent_electro_wrench = agent_agent_electro_wrenches[i_agent].clone();

        debug!("Computing electrostatic repulsion between agents and boundaries");
        let agent_boundary_electro_wrench = if enable_boundary_electro {
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
        let fluid_twist = fluid_twists[i_agent];
        let v_object = force_balance_v(
            sim_params,
            fluid_twist.0,
            f_propulsion + agent_agent_electro_wrench.force + agent_boundary_electro_wrench.force,
        );
        let dr_object = v_object * sim_params.dt + random_vector(rng, trans_diff_distr);
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
        let rot_rotational_diffusion =
            brownian::random_rotation(rng, &uniform_theta, &uniform_cos_phi, &rot_diff_distr);

        agent.seg.rotate(rot_rotational_diffusion * rot_object);

        // Apply periodic boundary condition.
        boundary::wrap_agent_inplace(&mut agent, &sim_params.boundaries);

        agent_summaries.push(AgentStepSummary {
            f_agent_electro: agent_agent_electro_wrench.force,
            torque_agent_electro: agent_agent_electro_wrench.torque,
            f_propulsion,
            torque_boundary_electro: agent_boundary_electro_wrench.torque,
            f_boundary_electro: agent_boundary_electro_wrench.force,
            v_fluid_back: fluid_twist.0,
            v_fluid_front: fluid_twist.0,
        });
    }

    // Upate time and step.
    sim_state.t += sim_params.dt;
    sim_state.step += 1;

    agent_summaries
}

pub fn run(
    conn: &mut diesel::PgConnection,
    sim_params: SimParams,
    mut sim_state: SimState,
    run_params: RunConfig,
) -> Result<(), Error> {
    let trans_diff_distr = brownian_distr(
        sim_params.agent_translational_diffusion_coefficient,
        sim_params.dt,
    );
    let rot_diff_distr = brownian_distr(
        sim_params.agent_rotational_diffusion_coefficient,
        sim_params.dt,
    );

    let rng = &mut rand::thread_rng();
    let gpu_fluid_context = cuda::fluid::CudaFluidContext::new(
        sim_state.agents.len() as u32,
        2 * sim_state.agents.len() as u32, // TODO: Assume two stokeslets per agent for now.
        sim_params.boundaries.clone(),
        32,
    );
    let gpu_electro_context = cuda::electro::CudaElectroContext::new(
        sim_state.agents.len() as u32,
        sim_params.boundaries.clone(),
        sim_params.agent_radius as f32,
        sim_params.agent_object_hertz_force_coefficient,
        5, // Number of approximation points for segment nearest-neighbours.
        32,
    );

    while sim_state.t < run_params.t_max {
        let summaries = update(
            &sim_params,
            &mut sim_state,
            trans_diff_distr,
            rot_diff_distr,
            &gpu_fluid_context,
            &gpu_electro_context,
            rng,
        );

        if sim_state.step % run_params.dstep_view == 0 {
            // Print time with resolution of 0.1ms.
            info!(
                "CHECKPOINT: step={:>8}, t = {:>9.4}",
                sim_state.step, sim_state.t
            );
            crate::db::write_checkpoint(conn, run_params.run_id, &sim_state, Some(summaries))?;
        }
    }
    Ok(())
}
