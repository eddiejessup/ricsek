use nalgebra::{zero, UnitVector3, Vector3};

pub fn electro_kinematics(
    repulser_normal: UnitVector3<f64>,
    overlap: f64,
    electro_coeff: f64,
) -> Vector3<f64> {
    // Electrostatics is only relevant when the objects' surfaces intersect.
    // Otherwise, there is no force.
    if overlap > 0.0 {
        // Force acts along repulsor normal.
        repulser_normal.scale(electro_coeff * overlap.powi(3).sqrt())
    } else {
        zero()
    }
}
