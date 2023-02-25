use ndarray::prelude::*;
use ndarray_rand::rand_distr::Normal;
use ndarray_rand::RandomExt;
use ricsek::runner::common::*;

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


pub fn run(sim_params: SimParams, mut sim_state: SimState, run_params: RunParams) {
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
    let run_id = 1;
    let dt_view = 0.02;
    let dt_chk = 2.0;
    // let run_params = RunParams {
    //     t_sim_max: 5.0,
    //     write_view: true,
    //     dstep_view: sim_params.to_steps(dt_view),
    //     write_chk: true,
    //     dstep_chk: sim_params.to_steps(dt_chk),
    //     run_id: run_id,
    // };

}
