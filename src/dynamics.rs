pub mod agent;
pub mod boundary;
pub mod brownian;
pub mod common;
pub mod obstacle;
pub mod stokes_solutions;

use crate::config::run::RunConfig;
use crate::config::setup::parameters::simulation::SimParams;
use crate::dynamics::agent::{agent_agents_electro, agent_agents_hydro};
use crate::dynamics::boundary::wrap;
use crate::dynamics::brownian::{rot_brownian_distr, trans_brownian_distr};
use crate::dynamics::obstacle::agent_obstacles_kinematics;
use crate::math::capsule::Capsule;
use crate::math::point::random_vector;
use crate::state::*;
use nalgebra::Rotation2;
use rand_distr::{Distribution, Normal};

pub fn update(
    sim_params: &SimParams,
    capsules: &[Capsule],
    sim_state: &mut SimState,
    trans_diff_distr: Normal<f64>,
    rot_diff_distr: Normal<f64>,
    rng: &mut rand::rngs::ThreadRng,
) {
    let agent_rs = sim_state.agents.iter().map(|a| a.r).collect::<Vec<_>>();
    for (i, agent) in sim_state.agents.iter_mut().enumerate() {
        // Fluid velocity and rotation.
        // Segment-agent velocity and rotation.
        let (mut v, om) = agent_obstacles_kinematics(
            agent,
            capsules,
            sim_params.agent_radius,
            sim_params.agent_aspect_ratio,
            sim_params.agent_obstacle_hydro_strength,
            sim_params.agent_object_hertz_velocity_coefficient,
        );

        // Agent-agent electrostatic force.
        let v_agents_electro = agent_agents_electro(
            i,
            agent.r,
            &agent_rs,
            sim_params.agent_radius,
            sim_params.agent_object_hertz_velocity_coefficient,
        );
        v += v_agents_electro;

        // Agent-agent hydrodynamic force.
        let v_agents_hydro = agent_agents_hydro(
            i,
            agent.r,
            &agent_rs,
            nalgebra::vector!(
                sim_params.agent_stresslet_force_longitudinal,
                sim_params.agent_stresslet_force_transverse
            ),
            sim_params.agent_stresslet_force_rotational,
        );
        v += v_agents_hydro;

        // Agent propulsion.
        v += agent.u.into_inner() * sim_params.agent_propulsion_speed;

        // Update agent position from velocity.
        agent.r += v * sim_params.dt;
        // Compute translational diffusion translation.
        agent.r += random_vector(rng, trans_diff_distr);

        // Apply periodic boundary condition.
        wrap(&mut agent.r, sim_params.l);

        let mut dth = om * sim_params.dt;
        // Compute rotational diffusion rotation.
        dth += &rot_diff_distr.sample(rng);
        // Perform the rotation.
        agent.u = Rotation2::new(dth) * agent.u;
    }

    // Upate time and step.
    sim_state.t += sim_params.dt;
    sim_state.step += 1;
}

pub fn run(
    conn: &mut diesel::PgConnection,
    sim_params: SimParams,
    obstacles: Vec<Capsule>,
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
        update(
            &sim_params,
            &obstacles,
            &mut sim_state,
            trans_diff_distr,
            rot_diff_distr,
            rng,
        );

        if sim_state.step % run_params.dstep_view == 0 {
            println!("CHECKPOINT: step={}, t = {}", sim_state.step, sim_state.t);
            crate::db::write_checkpoint(conn, run_params.run_id, &sim_state);
        }
    }
}
