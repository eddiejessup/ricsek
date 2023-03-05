use rand_distr::Normal;

pub fn trans_brownian_distr(d_diff: f64, dt: f64) -> Normal<f64> {
  let l = (2.0 * d_diff * dt).sqrt();
  Normal::new(0.0, l).unwrap()
}

pub fn rot_brownian_distr(d_diff: f64, dt: f64) -> Normal<f64> {
  let l = (2.0 * d_diff * dt).sqrt();
  Normal::new(0.0, l).unwrap()
}
