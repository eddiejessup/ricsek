use nalgebra::{vector, Point2, Vector2};

pub fn stokeslet_u(force: Vector2<f64>, fx: Point2<f64>, px: Point2<f64>) -> Vector2<f64> {
    let x = px - fx;
    let r = x.magnitude();
    (force / r) + (x * (force.dot(&x) / r.powi(3)))

    // U(r) = (1 / (8 * π * η)) * (F * r) / |r|^3
}

pub fn rotlet_u(torque: f64, fx: Point2<f64>, px: Point2<f64>) -> Vector2<f64> {
    let r = px - fx;
    let rm = r.magnitude();
    Vector2::new(r.y, -r.x) * torque / rm.powi(3)
}

pub fn doublet_u(strength: Vector2<f64>, fx: Point2<f64>, px: Point2<f64>) -> Vector2<f64> {
    let x = px - fx;
    let r = x.magnitude();
    -(strength / r.powi(3)) + (x * 3.0 * (strength.dot(&x) / r.powi(5)))
}

pub fn vpow(v: Vector2<f64>, n: i32) -> Vector2<f64> {
    v.map(|x| x.powi(n))
}

pub fn stresslet_u(
    s_diag: Vector2<f64>,
    s_off: f64,
    fx: Point2<f64>,
    px: Point2<f64>,
) -> Vector2<f64> {
    let r = px - fx;
    let rm_sq = r.magnitude_squared();

    let bs = vector!(r, vector!(r.y, r.x))
        .map(|a| r.component_mul(&vpow(a, 2)))
        .map(|a| (a / rm_sq.powi(2)).add_scalar(-rm_sq));

    s_diag.component_mul(&bs.x) + s_off * bs.y
}
