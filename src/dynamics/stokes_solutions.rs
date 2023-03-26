use crate::{
  math::point,
};

pub fn stokeslet_u(
  force: geo::Point,
  fx: geo::Point,
  px: geo::Point,
) -> geo::Point {
  let x = px - fx;
  let r = point::point_magnitude(x);
  (force / r) + (x * (force.dot(x) / r.powi(3)))
}

pub fn couplet_u(
  torque: f64,
  fx: geo::Point,
  px: geo::Point,
) -> geo::Point {
  let x = px - fx;
  let r = point::point_magnitude(x);
  geo::Point::new(x.y(), -x.x()) * torque / r.powi(3)
}

pub fn doublet_u(
  strength: geo::Point,
  fx: geo::Point,
  px: geo::Point,
) -> geo::Point {
  let x = px - fx;
  let r = point::point_magnitude(x);
  -(strength / r.powi(3)) + (x * 3.0 * (strength.dot(x) / r.powi(5)))
}

pub fn stresslet_u(
  stress_diag: geo::Point,
  stress_off: f64,
  fx: geo::Point,
  px: geo::Point,
) -> geo::Point {
  let r = px - fx;
  let rm = point::point_magnitude(r);

  // u_ij = (T_ij * r_j * r_i * r_j) / rm.pow(4) - rm.pow(2) * T_ij;
  let u_xx = (stress_diag.x() * r.x() * r.x() * r.x()) / rm.powi(4) - rm.powi(2) * stress_diag.x();
  let u_xy = (stress_off * r.y() * r.x() * r.y()) / rm.powi(4) - rm.powi(2) * stress_off;
  let u_yx = (stress_off * r.x() * r.y() * r.x()) / rm.powi(4) - rm.powi(2) * stress_off;
  let u_yy = (stress_diag.y() * r.y() * r.y() * r.y()) / rm.powi(4) - rm.powi(2) * stress_diag.y();

  let u_x = u_xx + u_xy;
  let u_y = u_yx + u_yy;

  geo::Point::new(u_x, u_y)
}
