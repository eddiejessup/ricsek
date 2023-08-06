use nalgebra::{UnitVector3, Vector3, zero};

pub fn electro_kinematics(
  y_unit: UnitVector3<f64>,
  overlap: f64,
  electro_coeff: f64,
) -> Vector3<f64> {
  // Electrostatics is only relevant when the objects' surfaces intersect.
  // Otherwise, there is no force.
  if overlap > 0.0 {
      // Negative means away from y_unit, meaning repulsion.
      -y_unit.into_inner() * electro_coeff * overlap.powi(3).sqrt()
    } else {
      zero()
  }
}
