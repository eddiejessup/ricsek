use nalgebra::{vector, Vector2};

// r is the vector pointing from the singularity to the point of interest.

fn perp(x: Vector2<f64>) -> Vector2<f64> {
  Vector2::new(x.y, -x.x)
}

// Source: https://sci-hub.ee/10.1017/S0022112075000614, Eq. 3a
pub fn stokeslet_u(f: Vector2<f64>, r: Vector2<f64>) -> Vector2<f64> {
    let rm = r.magnitude();
    (f / rm) + (r * (f.dot(&r) / rm.powi(3)))
}

pub fn stokeslet_vort(f: Vector2<f64>, r: Vector2<f64>) -> Vector2<f64> {
    let rm = r.magnitude();
    (f / rm) + (r * (f.dot(&r) / rm.powi(3)))
}

pub fn rotlet_u(torque: f64, r: Vector2<f64>) -> Vector2<f64> {
    let rm = r.magnitude();
    perp(r) * torque / rm.powi(3)
}

pub fn doublet_u(strength: Vector2<f64>, r: Vector2<f64>) -> Vector2<f64> {
    let rm = r.magnitude();
    -(strength / rm.powi(3)) + (r * 3.0 * (strength.dot(&r) / rm.powi(5)))
}

pub fn vpow(v: Vector2<f64>, n: i32) -> Vector2<f64> {
    v.map(|x| x.powi(n))
}

pub fn stresslet_u(s_diag: f64, s_off: f64, r: Vector2<f64>) -> Vector2<f64> {
    let rm_sq = r.magnitude_squared();

    vec![
        (vector![s_diag, -s_diag], r),
        (vector![s_off, s_off], vector![r.y, r.x]),
    ]
    .iter()
    .map(|(s, a)| {
        let b = r.component_mul(&vpow(*a, 2));
        // Compute b / r^4 - 1 / r^2
        let c = (b / rm_sq.powi(2)).add_scalar(-rm_sq);
        s.component_mul(&c)
    })
    .sum()
}

// u = p_mag (-1/r_mag^3 + 3(p_unit dot r)^2)/r_mag^5) r
pub fn stresslet_u_2(p: Vector2<f64>, r: Vector2<f64>) -> Vector2<f64> {
  let p_mag = p.norm();
  let p_hat = p / p_mag;
  let r_mag = r.norm();
  let r_hat = r / r_mag;

  let term1 = -1.0 / r_mag.powi(3);
  let term2 = 3.0 * (p_hat.dot(&r)).powi(2) / r_mag.powi(5);

  p_mag * (term1 + term2) * r
}
