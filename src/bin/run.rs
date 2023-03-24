use rand_distr::Distribution;
use ricsek::math::capsule::Capsule;
use ricsek::parameters::{RunParams, simulation::SimParams};
use ricsek::state::*;
use ricsek::dynamics::boundary::wrap;
use ricsek::dynamics::brownian::{rot_brownian_distr, trans_brownian_distr};
use ricsek::dynamics::obstacle::agent_obstacles_kinematics;
use ricsek::math::{random_coord, rotate_point_inplace};

pub fn run(
    conn: &mut diesel::PgConnection,
    sim_params: SimParams,
    capsules: Vec<Capsule>,
    mut sim_state: SimState,
    run_params: RunParams,
) {
    let trans_diff_distr = trans_brownian_distr(sim_params.d_trans_diff, sim_params.dt);
    let rot_diff_distr = rot_brownian_distr(sim_params.d_rot_diff, sim_params.dt);

    let rng = &mut rand::thread_rng();

    while sim_state.t < run_params.t_max {
        sim_state.agents.iter_mut().for_each(|agent| {
            // Fluid velocity and rotation.
            // Segment-agent velocity and rotation.
            let (mut v, om) = agent_obstacles_kinematics(
                agent,
                &capsules,
                sim_params.ag_radius,
                sim_params.aspect_ratio,
                sim_params.k_repulse,
                sim_params.seg_v_overlap_coeff,
            );

            // Agent propulsion.
            v += agent.u * sim_params.ag_v_propulse;

            // Update agent position from velocity.
            agent.r += v * sim_params.dt;
            // Compute translational diffusion translation.
            agent.r += random_coord(rng, trans_diff_distr).into();

            // Apply periodic boundary condition.
            wrap(&mut agent.r, sim_params.l);

            let mut dth = om * sim_params.dt;
            // Compute rotational diffusion rotation.
            dth += &rot_diff_distr.sample(rng);
            // Perform the rotation.
            rotate_point_inplace(&mut agent.u, dth);
        });

        // Upate time and step.
        sim_state.t += sim_params.dt;
        sim_state.step += 1;

        if sim_state.step % run_params.dstep_view == 0 {
            println!("CHECKPOINT: step={}, t = {}", sim_state.step, sim_state.t);
            ricsek::db::write_checkpoint(conn, run_params.run_id, &sim_state);
        }
    }
}

fn main() {
    let dt_view = 0.02;
    let t_max = 12.0;

    let conn = &mut ricsek::db::establish_connection();

    let run_id = ricsek::db::read_latest_run_id(conn);
    println!("Running run_id: {}", run_id);
    let sim_setup = ricsek::db::read_run(conn, run_id);
    let sim_state = ricsek::db::read_latest_checkpoint(conn, run_id);
    println!("Running from step {}, t={}s", sim_state.step, sim_state.t);

    let run_params = RunParams {
        t_max,
        dstep_view: sim_setup.params.to_steps(dt_view),
        run_id,
    };

    run(
        conn,
        sim_setup.params,
        sim_setup.capsules,
        sim_state,
        run_params,
    );
    println!("Done!");
}
