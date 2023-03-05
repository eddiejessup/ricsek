use ndarray::prelude::*;
use ndarray_rand::rand_distr::Normal;
use ndarray_rand::RandomExt;

// let rot_diff_distr = Normal::new(0.0, sim_params.len_rot_diff()).unwrap();

pub fn trans_brownian_distr(d_diff: f64, dt: f64) -> Normal<f64> {
  let l = (2.0 * d_diff * dt).sqrt();
  Normal::new(0.0, l).unwrap()
}

pub fn rot_brownian_distr(d_diff: f64, dt: f64) -> Normal<f64> {
  let l = (2.0 * d_diff * dt).sqrt();
  Normal::new(0.0, l).unwrap()
}

pub fn trans_brownian_noise(n: usize, distr: Normal<f64>) -> Array2<f64> {
    Array::random((n, 2), distr)
}

pub fn rot_brownian_noise(n: usize, distr: Normal<f64>) -> Array1<f64> {
  Array::random(n, distr)
}
