use ndarray::Array2;

pub fn wrap(r: &mut Array2<f64>, l: f64) {
  let l_half = l * 0.5;
  r.mapv_inplace(|x| {
      if x < -l_half {
          x + l
      } else if x > l_half {
          x - l
      } else {
        x
      }
  });
}
