use bevy::prelude::*;
use nalgebra::Vector3;

pub fn nalgebra_to_glam_vec(v: &Vector3<f64>) -> Vec3 {
    Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}

pub fn increment_step(cur_i: usize, backward: bool, maxi: usize, stepsize: usize) -> usize {
  if backward {
      if cur_i > stepsize {
          cur_i - stepsize
      } else {
          0
      }
  } else {
      (cur_i + stepsize).min(maxi)
  }
}
