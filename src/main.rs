extern crate ndarray;

use ndarray::prelude::*;
use ndarray::Zip;
use ndarray_rand::rand_distr::Normal;
use ndarray_rand::rand_distr::Uniform;
use ndarray_rand::RandomExt;

struct SimState {
    u_p: Array2<f64>,
    r: Array2<f64>,
    t_sim: f64,
    step_sim: usize,
}

impl SimState {
    fn new(u_p: Array2<f64>, r: Array2<f64>) -> SimState {
        SimState {
            u_p,
            r,
            t_sim: 0.0,
            step_sim: 0,
        }
    }
}

struct SimParams {
    dt_sim: f64,
    l: f64,
    v_propulse: f64,
    segments: Vec<Array2<f64>>,
    segment_repulse_v_0: f64,
    segment_repulse_d_0: f64,
    segment_align_omega_0: f64,
    segment_align_d_0: f64,
    ag_repulse_d_0: f64,
    ag_repulse_v_0: f64,
    d_trans_diff: f64,
    d_rot_diff: f64,
    n: usize,
}

impl SimParams {
    fn to_steps(&self, t: f64) -> usize {
        (t / self.dt_sim).ceil() as usize
    }

    fn len_rot_diff(&self) -> f64 {
        (2.0 * self.d_rot_diff * self.dt_sim).sqrt()
    }

    fn len_trans_diff(&self) -> f64 {
        (2.0 * self.d_trans_diff * self.dt_sim).sqrt()
    }

    fn l_half(&self) -> f64 {
        self.l * 0.5
    }
}

struct RunParams {
    t_sim_max: f64,
    write_view: bool,
    dstep_view: usize,
    write_chk: bool,
    dstep_chk: usize,
    run_id: usize,
}

fn rotate_2d_vec_inplace(v: &mut ArrayViewMut1<f64>, theta: f64) {
    let v0 = *v.get(0).unwrap();
    let v1 = *v.get(1).unwrap();
    *v.get_mut(0).unwrap() = v0 * theta.cos() - v1 * theta.sin();
    *v.get_mut(1).unwrap() = v0 * theta.sin() + v1 * theta.cos();
}

fn rotate_2d_vecs_inplace(v: &mut ArrayViewMut2<f64>, theta: ArrayView1<f64>) {
    Zip::from(v.rows_mut())
        .and(theta)
        .for_each(|mut x, theta| rotate_2d_vec_inplace(&mut x, *theta));
}

fn wrap(d: &mut Array2<f64>, l: f64) {
    let l_half = l * 0.5;
    d.mapv_inplace(|mut x| {
        if x < -l_half {
            x += l
        } else if x > l_half {
            x -= l
        };
        x
    });
}

fn run(sim_params: SimParams, mut sim_state: SimState, run_params: RunParams) {
    let trans_diff_distr = Normal::new(0.0, sim_params.len_trans_diff()).unwrap();
    let rot_diff_distr = Normal::new(0.0, sim_params.len_rot_diff()).unwrap();

    while sim_state.t_sim < run_params.t_sim_max {
        // Compute environment and agent variables.

        // Update agent position.
        let mut dr = Array::zeros((sim_params.n, 2));
        // Compute propulsion translation.
        // Overall agent velocity.
        let mut v = Array::zeros((sim_params.n, 2));
        // Agent propulsion.
        v.scaled_add(sim_params.v_propulse, &sim_state.u_p);
        // Update agent position.
        dr.scaled_add(sim_params.dt_sim, &v);
        // Compute translational diffusion translation.
        dr += &Array::random((sim_params.n, 2), trans_diff_distr);

        // Perform the translation.
        sim_state.r += &dr;
        // Apply periodic boundary condition.
        wrap(&mut (sim_state.r), sim_params.l);

        // Update agent direction.
        let mut dth = Array::zeros(sim_params.n);
        // Compute rotational diffusion rotation.
        dth += &Array::random((sim_params.n,), rot_diff_distr);

        // Compute torque rotation.
        let omega = Array::zeros(sim_params.n);
        dth.scaled_add(sim_params.dt_sim, &omega);

        // Perform the rotation.
        rotate_2d_vecs_inplace(&mut sim_state.u_p.view_mut(), dth.view());

        // (C). Update environment

        // Upate time and step.
        sim_state.t_sim += sim_params.dt_sim;
        sim_state.step_sim += 1
    }
}

fn main() {
    let sim_params = SimParams {
        dt_sim: 0.1,
        l: 1.0,
        v_propulse: 0.1,
        segments: Vec::new(),
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

    let run_id = 1;
    let dt_view = 0.02;
    let dt_chk = 2.0;
    let run_params = RunParams {
        t_sim_max: 5.0,
        write_view: true,
        dstep_view: sim_params.to_steps(dt_view),
        write_chk: true,
        dstep_chk: sim_params.to_steps(dt_chk),
        run_id: run_id,
    };

    let sim_state = SimState::new(u_p, r);
    run(sim_params, sim_state, run_params);
}
