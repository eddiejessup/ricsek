use ndarray::prelude::*;
use ndarray_rand::rand_distr::Uniform;
use ndarray_rand::RandomExt;
use ricsek::common::*;

fn main() {
    let sim_params = SimParams {
        dt_sim: 0.1,
        l: 1.0,
        v_propulse: 0.1,
        segments: Array3::default((0, 2, 2)),
        segment_repulse_v_0: 1.0,
        segment_repulse_d_0: 0.01,
        segment_align_omega_0: 50.0,
        segment_align_d_0: 0.01,
        ag_repulse_d_0: 1.0,
        ag_repulse_v_0: 1.0,
        d_trans_diff: 0.00002,
        d_rot_diff: 0.1,
        n: 10,
    };

    let r = Array::random(
        (sim_params.n, 2),
        Uniform::new(-sim_params.l * 0.5, sim_params.l * 0.5),
    );

    let mut u_p = Array::<f64, Ix2>::zeros((sim_params.n, 2));
    u_p.slice_mut(s![.., 0]).fill(1.0);
    rotate_2d_vecs_inplace(
        &mut u_p.view_mut(),
        Array::random(
            sim_params.n,
            Uniform::new(-std::f64::consts::PI, std::f64::consts::PI),
        )
        .view(),
    );

    let sim_state = SimState::new(u_p, r);

    let connection = &mut ricsek::db::establish_connection();

    let run_id = ricsek::db::initialize_run(connection, &sim_params);
    ricsek::db::write_checkpoint(connection, run_id, &sim_state);
}
