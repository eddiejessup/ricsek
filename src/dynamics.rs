pub mod agent;
pub mod boundary;
pub mod brownian;
pub mod electro;
pub mod stokes_solutions;

use std::f64::consts::PI;

use crate::config::run::RunConfig;
use crate::config::setup::parameters::simulation::SimParams;
use crate::dynamics::agent::{agent_agents_electro, agent_agents_hydro};
use crate::dynamics::brownian::{rot_brownian_distr, trans_brownian_distr};
use crate::math::point::random_vector;
use crate::state::*;
use nalgebra::Vector3;
use rand_distr::{Normal, Uniform};

pub fn update(
    sim_params: &SimParams,
    sim_state: &mut SimState,
    trans_diff_distr: Normal<f64>,
    rot_diff_distr: Normal<f64>,
    rng: &mut rand::rngs::ThreadRng,
) -> Vec<AgentStepSummary> {
    let uniform_theta = Uniform::new(0.0, 2.0 * PI);
    let uniform_cos_phi = Uniform::new(-1.0, 1.0);

    let agent_rs = sim_state.agents.iter().map(|a| a.r).collect::<Vec<_>>();

    let mut agent_summaries: Vec<AgentStepSummary> = vec![];
    for (i, agent) in sim_state.agents.iter_mut().enumerate() {
        let r1c_r2cs = agent_rs
            .iter()
            .map(|r2| boundary::wrap_vec(r2 - agent.r, &sim_params.boundaries))
            .collect::<Vec<_>>();

        // Fluid velocity and rotation.

        // Agent-agent electrostatic repulsion.
        let v_agent_electro = agent_agents_electro(
            i,
            &r1c_r2cs,
            sim_params.agent_radius,
            sim_params.agent_object_hertz_velocity_coefficient,
        );

        // Agent-agent hydrodynamic force.
        let v_agent_hydro = agent_agents_hydro(
            i,
            &r1c_r2cs,
            agent.u.scale(sim_params.agent_agent_hydro_a),
            agent.u.scale(sim_params.agent_agent_hydro_b),
        );

        // Agent propulsion.
        let v_propulsion = agent.u.into_inner() * sim_params.agent_propulsion_speed;

        // Agent-boundary electrostatic repulsion.
        let v_boundary_electro = boundary::agent_boundary_electro(
            agent.r,
            &sim_params.boundaries,
            sim_params.agent_radius,
            sim_params.agent_object_hertz_velocity_coefficient,
        );

        // Agent-singularity force.
        let v_singularity = sim_params
            .singularities
            .iter()
            .map(|singularity| singularity.eval(agent.r))
            .sum::<Vector3<f64>>();

        let v_net = v_agent_electro + v_agent_hydro + v_propulsion + v_boundary_electro + v_singularity;

        // Update agent position from velocity.
        agent.r += v_net * sim_params.dt;
        // Compute translational diffusion translation.
        agent.r += random_vector(rng, trans_diff_distr);

        // Apply periodic boundary condition.
        agent.r = boundary::wrap(agent.r, &sim_params.boundaries);

        // Compute rotational diffusion rotation.
        let rot = brownian::random_rotation(rng, &uniform_theta, &uniform_cos_phi, &rot_diff_distr);
        // Perform the rotation.
        agent.u = rot * agent.u;

        agent_summaries.push(AgentStepSummary {
          v_agent_electro,
          v_agent_hydro,
          v_propulsion,
          v_boundary_electro,
          v_singularity,
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
) {
    let trans_diff_distr = trans_brownian_distr(
        sim_params.agent_translational_diffusion_coefficient,
        sim_params.dt,
    );
    let rot_diff_distr = rot_brownian_distr(
        sim_params.agent_rotational_diffusion_coefficient,
        sim_params.dt,
    );

    let rng = &mut rand::thread_rng();

    while sim_state.t < run_params.t_max {
        let summaries = update(
            &sim_params,
            &mut sim_state,
            trans_diff_distr,
            rot_diff_distr,
            rng,
        );

        if sim_state.step % run_params.dstep_view == 0 {
            println!("CHECKPOINT: step={}, t = {}", sim_state.step, sim_state.t);
            crate::db::write_checkpoint(conn, run_params.run_id, &sim_state, Some(summaries));
        }
    }
}
