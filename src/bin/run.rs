use ndarray::prelude::*;
use ricsek::common::*;
use ricsek::common::params::{RunParams, SimParams};
use ricsek::dynamics::segment::n_agent_n_segments_kinematics;
use ricsek::dynamics::boundary::wrap;
use ricsek::dynamics::brownian::{trans_brownian_noise, rot_brownian_noise, trans_brownian_distr, rot_brownian_distr};
use ricsek::math::{LineSegment, rotate_2d_vecs_inplace};

pub fn run(
    conn: &mut diesel::PgConnection,
    sim_params: SimParams,
    segments: Vec<LineSegment>,
    mut sim_state: SimState,
    run_params: RunParams,
) {
    let trans_diff_distr = trans_brownian_distr(sim_params.d_trans_diff, sim_params.dt);
    let rot_diff_distr = rot_brownian_distr(sim_params.d_rot_diff, sim_params.dt);

    while sim_state.t < run_params.t_max {
        // Compute environment and agent variables.

        // Initial agent position and orientation changes.
        let mut dr = Array::zeros((sim_params.n, 2));
        let mut dth = Array::zeros(sim_params.n);

        // Initialise memoryless properties.
        //   - Agent velocity.
        let mut v = Array::zeros((sim_params.n, 2));
        //   - Agent rotation rate.
        let mut om = Array::zeros(sim_params.n);

        // Agent propulsion.
        v.scaled_add(
            sim_params.ag_f_propulse * sim_params.ag_trans_mobility,
            &sim_state.u_p,
        );
        // Segment-agent velocity and rotation.
        let (v_seg, om_seg) = n_agent_n_segments_kinematics(
            sim_state.r.view(),
            sim_state.u_p.view(),
            &segments,
            sim_params.k_repulse,
            sim_params.aspect_ratio,
        );
        v += &v_seg;
        om += &om_seg;

        // Update agent position from velocity.
        dr.scaled_add(sim_params.dt, &v);
        // Compute translational diffusion translation.
        dr += &trans_brownian_noise(sim_params.n, trans_diff_distr);
        // Perform the translation.
        sim_state.r += &dr;
        // Apply periodic boundary condition.
        wrap(&mut sim_state.r, sim_params.l);

        dth.scaled_add(sim_params.dt, &om);
        // Compute rotational diffusion rotation.
        dth += &rot_brownian_noise(sim_params.n, rot_diff_distr);
        // Perform the rotation.
        rotate_2d_vecs_inplace(&mut sim_state.u_p.view_mut(), dth.view());

        // (C). Update environment

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
