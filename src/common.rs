use ndarray::prelude::*;
use ndarray::Zip;

pub struct SimState {
    pub u_p: Array2<f64>,
    pub r: Array2<f64>,
    pub t: f64,
    pub step: usize,
}

impl SimState {
    pub fn new(u_p: Array2<f64>, r: Array2<f64>) -> SimState {
        SimState {
            u_p,
            r,
            t: 0.0,
            step: 0,
        }
    }
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct SimParams {
    pub dt_sim: f64,
    pub l: f64,
    pub v_propulse: f64,
    pub segments: Array3<f64>,
    pub segment_repulse_v_0: f64,
    pub segment_repulse_d_0: f64,
    pub segment_align_omega_0: f64,
    pub segment_align_d_0: f64,
    pub ag_repulse_d_0: f64,
    pub ag_repulse_v_0: f64,
    pub d_trans_diff: f64,
    pub d_rot_diff: f64,
    pub n: usize,
}

impl SimParams {
    pub fn to_steps(&self, t: f64) -> usize {
        (t / self.dt_sim).ceil() as usize
    }

    pub fn len_rot_diff(&self) -> f64 {
        (2.0 * self.d_rot_diff * self.dt_sim).sqrt()
    }

    pub fn len_trans_diff(&self) -> f64 {
        (2.0 * self.d_trans_diff * self.dt_sim).sqrt()
    }

    pub fn l_half(&self) -> f64 {
        self.l * 0.5
    }
}

pub struct RunParams {
    pub t_max: f64,
    pub dstep_view: usize,
    pub run_id: usize,
}

pub fn rotate_2d_vec_inplace(v: &mut ArrayViewMut1<f64>, theta: f64) {
    let v0 = *v.get(0).unwrap();
    let v1 = *v.get(1).unwrap();
    *v.get_mut(0).unwrap() = v0 * theta.cos() - v1 * theta.sin();
    *v.get_mut(1).unwrap() = v0 * theta.sin() + v1 * theta.cos();
}

pub fn rotate_2d_vecs_inplace(v: &mut ArrayViewMut2<f64>, theta: ArrayView1<f64>) {
    Zip::from(v.rows_mut())
        .and(theta)
        .for_each(|mut x, theta| rotate_2d_vec_inplace(&mut x, *theta));
}
