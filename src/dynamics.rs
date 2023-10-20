pub mod agent;
pub mod boundary;
pub mod brownian;
pub mod electro;
pub mod stokes_solutions;

use std::f64::consts::PI;

use crate::config::run::RunConfig;
use crate::config::setup::parameters::simulation::SimParams;
use crate::dynamics::brownian::{rot_brownian_distr, trans_brownian_distr};
use crate::geometry::point::random_vector;
use crate::state::*;
use diesel::result::Error;
use log::{debug, info};
use nalgebra::{Point3, Rotation3, UnitVector3, Vector3};
use rand_distr::{Normal, Uniform};

use self::agent::{agents_agents_electro, agents_fluid_v};

pub fn fluid_v_at(
    sim_params: &SimParams,
    r: Point3<f64>,
    i_agent: usize,
    agents: &[Agent],
) -> Vector3<f64> {
    // Singularity contribution to flow.
    let v_fluid_singularity = sim_params
        .singularities
        .iter()
        .map(|singularity| singularity.eval(r))
        .sum::<Vector3<f64>>();
    // Other-agent contribution to flow.
    let v_fluid_agents = agents_fluid_v(r, i_agent, agents, sim_params.agent_propulsion_force);
    v_fluid_singularity + v_fluid_agents
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
    rng: &mut rand::rngs::ThreadRng,
) -> Vec<AgentStepSummary> {
    let uniform_theta = Uniform::new(0.0, 2.0 * PI);
    let uniform_cos_phi = Uniform::new(-1.0, 1.0);

    let agents = sim_state.agents.clone();

    // Agent-agent electrostatic repulsion.
    let agent_agent_electro_wrenches: Vec<_> = agents_agents_electro(
        &sim_state.agents,
        sim_params.agent_radius,
        sim_params.agent_object_hertz_force_coefficient,
    );

    let mut agent_summaries: Vec<AgentStepSummary> = vec![];
    for (i_agent, mut agent) in sim_state.agents.iter_mut().enumerate() {
        // Fluid velocity and rotation.

        // Agent propulsion.
        // We know there's no torque because it acts along the agent's axis.
        debug!("Computing propulsion force");
        let f_propulsion = agent.u().into_inner() * sim_params.agent_propulsion_force;

        debug!("Computing electrostatic repulsion between agents and agents");
        let (f_agent_electro, torque_agent_electro) = agent_agent_electro_wrenches[i_agent];

        debug!("Computing electrostatic repulsion between agents and boundaries");
        let (f_boundary_electro, torque_boundary_electro) = boundary::boundary_electro(
            agent,
            sim_params.agent_radius,
            &sim_params.boundaries,
            sim_params.agent_object_hertz_force_coefficient,
        );

        debug!("Doing force balance calculation for linear velocity");
        let v_fluid = fluid_v_at(sim_params, agent.seg.centroid(), i_agent, &agents);
        let v_object = force_balance_v(
            sim_params,
            v_fluid,
            f_propulsion + f_agent_electro + f_boundary_electro,
        );
        let dr_object = v_object * sim_params.dt;
        // Compute translational diffusion translation.
        agent
            .seg
            .translate(random_vector(rng, trans_diff_distr) + dr_object);

        // Assume fluid has no vorticity.
        debug!("Doing force balance calculation for angular velocity");
        let omega_fluid = nalgebra::zero();
        let omega_object = force_balance_omega(
            sim_params,
            omega_fluid,
            torque_agent_electro + torque_boundary_electro,
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
            f_agent_electro,
            torque_agent_electro,
            f_propulsion,
            torque_boundary_electro,
            f_boundary_electro,
            v_fluid_back: v_fluid,
            v_fluid_front: v_fluid,
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
            info!("CHECKPOINT: step={}, t = {}", sim_state.step, sim_state.t);
            crate::db::write_checkpoint(conn, run_params.run_id, &sim_state, Some(summaries))?;
        }
    }
    Ok(())
}
