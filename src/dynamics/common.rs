pub fn electro_kinematics(
  y_unit: geo::Point,
  overlap: f64,
  electro_coeff: f64,
) -> geo::Point {
  // Electrostatics is only relevant when the objects' surfaces intersect.
  // Otherwise, there is no force.
  if overlap > 0.0 {
      // Negative means away from y_unit, meaning repulsion.
      -y_unit * electro_coeff * overlap.powi(3).sqrt()
    } else {
      geo::Point(geo::Coord { x: 0.0, y: 0.0 })
  }
}
