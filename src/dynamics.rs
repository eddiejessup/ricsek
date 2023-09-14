pub mod agent;
pub mod boundary;
pub mod brownian;
pub mod electro;
pub mod stokes_solutions;

use std::f64::consts::PI;

use crate::config::run::RunConfig;
use crate::config::setup::parameters::simulation::SimParams;
use crate::dynamics::agent::agent_agents_electro;
use crate::dynamics::brownian::{rot_brownian_distr, trans_brownian_distr};
use crate::geometry::point::random_vector;
use crate::geometry::sphere::Sphere;
use crate::state::*;
use log::{debug, info};
use nalgebra::{Point3, Vector3, UnitVector3, Rotation3};
use rand_distr::{Normal, Uniform};

pub fn fluid_v_at(sim_params: &SimParams, r: Point3<f64>) -> Vector3<f64> {
    // Singularity contribution to flow.
    let v_fluid_singularity = sim_params
        .singularities
        .iter()
        .map(|singularity| singularity.eval(r))
        .sum::<Vector3<f64>>();
    // Other-agent contribution to flow.
    let v_fluid_agents = Vector3::zeros();
    v_fluid_singularity + v_fluid_agents
}

pub fn force_balance_v(sim_params: &SimParams, r: &mut Point3<f64>, f: Vector3<f64>) {
    // Agent-boundary electrostatic repulsion.
    let f_boundary_electro = boundary::boundary_electro(
        &Sphere {
            point: *r,
            radius: sim_params.agent_radius,
        },
        &sim_params.boundaries,
        sim_params.agent_object_hertz_force_coefficient,
    );

    let f_tot = f + f_boundary_electro;
    let v = fluid_v_at(sim_params, *r) + f_tot * sim_params.agent_translational_mobility;
    debug!("Specific forces: {}", 1e12 * f);
    debug!("Boundary electro force: {}", 1e12 * f_boundary_electro);
    debug!("Total force: {}", 1e12 * f_tot);
    debug!("Body part velocity: {}", 1e6 * v);
    // Update agent position from velocity.
    let dr = v * sim_params.dt;
    debug!("Body part displacement: {}", 1e6 * dr);
    *r += dr;
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

    let mut agent_summaries: Vec<AgentStepSummary> = vec![];
    for (i, mut agent) in sim_state.agents.iter_mut().enumerate() {
        // Fluid velocity and rotation.

        // Agent propulsion.
        let f1_propulsion = agent.u().into_inner() * sim_params.agent_propulsion_force;
        let f1_spring = agent.r1_stretch_force(
            sim_params.agent_inter_sphere_length,
            sim_params.agent_linear_spring_stiffness,
        );
        debug!("Spring force on front-end: {}", -1e12 * f1_spring);
        // Agent-agent electrostatic repulsion.
        let (f_agent_electro, torque_agent_electro) = agent_agents_electro(
            agent,
            i,
            &agents,
            sim_params.agent_radius,
            sim_params.agent_object_hertz_force_coefficient,
        );

        debug!("Doing force balance calculation for back end");
        force_balance_v(
            sim_params,
            &mut agent.seg.start,
            f1_propulsion + f1_spring + f_agent_electro,
        );
        debug!("Doing force balance calculation for front end");
        force_balance_v(sim_params, &mut agent.seg.end, -f1_spring + f_agent_electro);

        let angular_v_electro = torque_agent_electro * sim_params.agent_rotational_mobility;
        let theta_electro = angular_v_electro * sim_params.dt;
        let rot_electro =
            Rotation3::from_axis_angle(&UnitVector3::new_normalize(theta_electro), theta_electro.norm());

        // Compute translational diffusion translation.
        agent.seg.translate(random_vector(rng, trans_diff_distr));

        // Compute rotational diffusion rotation.
        let rot_rotational_diffusion =
            brownian::random_rotation(rng, &uniform_theta, &uniform_cos_phi, &rot_diff_distr);

        agent.seg.rotate(rot_rotational_diffusion * rot_electro);

        // Apply periodic boundary condition.
        boundary::wrap_agent_inplace(&mut agent, &sim_params.boundaries);

        agent_summaries.push(AgentStepSummary {
            f_agent_electro,
            torque_agent_electro,
            f_agent_hydro: Vector3::zeros(),
            f_propulsion: f1_propulsion,
            f_boundary_electro: Vector3::zeros(),
            f_singularity: Vector3::zeros(),
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
            info!("CHECKPOINT: step={}, t = {}", sim_state.step, sim_state.t);
            crate::db::write_checkpoint(conn, run_params.run_id, &sim_state, Some(summaries));
        }
    }
}
