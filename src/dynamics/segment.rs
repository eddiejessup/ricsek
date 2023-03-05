use ndarray::prelude::*;
use ndarray::Array2;
use crate::math::*;

fn one_agent_1_segment_kinematics(
  r: ArrayView1<f64>,
  u: ArrayView1<f64>,
  segment: &LineSegment,
  k: f64,
  aspect_ratio: f64,
) -> (Array1<f64>, f64) {
  // https://arxiv.org/pdf/0806.2898.pdf
  // The velocity component of the swimmer towards the segment:
  //   v_y(θ, y) = (−3p / 64πηy^2) * (1 − 3 cos^2(θ)).
  // Where:
  // - p is the dipole strength
  // - θ is the angle between the segment normal and the swimmer orientation.
  // - y is the distance between the swimmer and the segment.

  // The nearest point on the segment to the swimmer.
  // println!("r: {:?}", 1e6 * &r);
  // println!("u (orientation): {:?}", u);
  let r_seg = segment.nearest_point(r);
  // println!("r_seg: {:?}", 1e6 * &r_seg);

  // The vector from the segment to the swimmer.
  let y_vec = &r - &r_seg;
  // println!("y_vec: {:?}", 1e6 * &y_vec);
  // The distance from the segment to the swimmer.
  let y = norm_one_vec(&y_vec);
  // println!("y: {:?}", 1e6 * &y);

  // To find the angle between the swimmer's direction and the segment normal:
  //   cos(θ) = y_vec_unit ⋅ u
  let y_vec_unit = y_vec / y;
  // println!("y_vec_unit: {:?}", y_vec_unit);
  let cos_th = y_vec_unit.dot(&u);
  // println!("cos_th: {:?}", cos_th);
  // println!("th: {:?} degrees", cos_th.acos().to_degrees());

  // The velocity.
  let v = -(k / y.powi(2)) * (1.0 - 3.0 * cos_th.powi(2));
  // println!("v: {:?}", 1e6 * &v);

  let ar_factor = (aspect_ratio.powi(2) - 1.0) / (2.0 * (aspect_ratio.powi(2) + 1.0));
  // println!("ar_factor: {:?}", &ar_factor);
  let sin_th = (1.0 - cos_th.powi(2)).sqrt();
  // println!("sin_th: {:?}", &sin_th);
  let om = (-k * cos_th * sin_th / y.powi(3)) * (1.0 + ar_factor * (1.0 + cos_th.powi(2)));
  // println!("om: {:?} degrees/s", &om.to_degrees());

  (v * &y_vec_unit, om)
}

fn n_agent_1_segment_kinematics(
  r: ArrayView2<f64>,
  u: ArrayView2<f64>,
  segment: &LineSegment,
  k: f64,
  aspect_ratio: f64,
) -> (Array2<f64>, Array1<f64>) {
  let mut v = Array::zeros((r.nrows(), 2));
  let mut om: Array1<f64> = Array::zeros(r.nrows());
  for i in 0..r.nrows() {
      let (v_i, om_i) = one_agent_1_segment_kinematics(r.row(i), u.row(i), segment, k, aspect_ratio);
      v.row_mut(i).assign(&v_i);
      om[i] = om_i;
  }
  (v, om)
}

pub fn n_agent_n_segments_kinematics(
  r: ArrayView2<f64>,
  u: ArrayView2<f64>,
  segments: &Vec<LineSegment>,
  k: f64,
  aspect_ratio: f64,
) -> (Array2<f64>, Array1<f64>) {
let mut v = Array::zeros((r.nrows(), 2));
let mut om: Array1<f64> = Array::zeros(r.nrows());
for segment in segments {
  let (v1, om1) = n_agent_1_segment_kinematics(
      r,
      u,
      segment,
      k,
      aspect_ratio,
  );
  v += &v1;
  om += &om1;
}
(v, om)
}
