use nalgebra::{vector, Vector2};

// r is the vector pointing from the singularity to the point of interest.

// U(r) = (1 / (8 * π * η)) * (F * r) / |r|^3
pub fn stokeslet_u(force: Vector2<f64>, r: Vector2<f64>) -> Vector2<f64> {
    let rm = r.magnitude();
    (force / rm) + (r * (force.dot(&r) / rm.powi(3)))
}

pub fn rotlet_u(torque: f64, r: Vector2<f64>) -> Vector2<f64> {
    let rm = r.magnitude();
    Vector2::new(r.y, -r.x) * torque / rm.powi(3)
}

pub fn doublet_u(strength: Vector2<f64>, r: Vector2<f64>) -> Vector2<f64> {
    let rm = r.magnitude();
    -(strength / rm.powi(3)) + (r * 3.0 * (strength.dot(&r) / rm.powi(5)))
}

pub fn vpow(v: Vector2<f64>, n: i32) -> Vector2<f64> {
    v.map(|x| x.powi(n))
}

pub fn stresslet_u(s_diag: Vector2<f64>, s_off: f64, r: Vector2<f64>) -> Vector2<f64> {
    // Get the vector from the singularity to the point of interest.
    let rm_sq = r.magnitude_squared();

    let bs = vector!(r, vector!(r.y, r.x))
        .map(|a| r.component_mul(&vpow(a, 2)))
        .map(|a| (a / rm_sq.powi(2)).add_scalar(-rm_sq));

    s_diag.component_mul(&bs.x) + s_off * bs.y
}
