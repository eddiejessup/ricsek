use rand_distr::Distribution;
use ricsek::common::params::{RunParams, SimParams};
use ricsek::common::*;
use ricsek::dynamics::boundary::wrap;
use ricsek::dynamics::brownian::{rot_brownian_distr, trans_brownian_distr};
use ricsek::dynamics::segment::agent_segments_kinematics;
use ricsek::math::{random_point, rotate_point_inplace};

pub fn run(
    conn: &mut diesel::PgConnection,
    sim_params: SimParams,
    segments: Vec<geo::Line>,
    mut sim_state: SimState,
    run_params: RunParams,
) {
    let trans_diff_distr = trans_brownian_distr(sim_params.d_trans_diff, sim_params.dt);
    let rot_diff_distr = rot_brownian_distr(sim_params.d_rot_diff, sim_params.dt);

    let rng = &mut rand::thread_rng();

    while sim_state.t < run_params.t_max {
        // Compute environment and agent variables.

        sim_state.agents.iter_mut().for_each(|agent| {
            let mut dth: f64 = 0.0;

            // Initialise memoryless properties.
            //   - Agent velocity.
            let mut v: geo::Point = (0.0, 0.0).into();
            //   - Agent rotation rate.
            let mut om: f64 = 0.0;

            // Agent propulsion.
            v += agent.u * sim_params.ag_trans_mobility * sim_params.ag_f_propulse;

            // Segment-agent velocity and rotation.
            let (v_seg, om_seg) = agent_segments_kinematics(
                agent,
                &segments,
                sim_params.k_repulse,
                sim_params.aspect_ratio,
            );
            v += v_seg;
            om += om_seg;

            // Update agent position from velocity.
            agent.r += v * sim_params.dt;
            // Compute translational diffusion translation.
            agent.r += random_point(rng, trans_diff_distr);

            // Apply periodic boundary condition.
            wrap(&mut agent.r, sim_params.l);

            dth += om * sim_params.dt;
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
    let t_max = 10.0;

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
        sim_setup.segments,
        sim_state,
        run_params,
    );
    println!("Done!");
}
