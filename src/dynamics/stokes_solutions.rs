use core::panic;

use nalgebra::Vector3;

// r is the vector pointing from the singularity to the point of interest.

fn safe_magnitude(r: Vector3<f64>) -> f64 {
    let rm = r.magnitude();
    if rm < 1e-10 {
        panic!("r is too small: {}", rm);
    } else {
        rm
    }
}

// Source: https://sci-hub.ee/10.1017/S0022112075000614, Eq. 3a
// Map alpha, beta, gamma, delta to a, b, c, d.
pub fn stokeslet_u(a: Vector3<f64>, r: Vector3<f64>) -> Vector3<f64> {
    let rm = safe_magnitude(r);
    (a / rm) + (a.dot(&r) * r / rm.powi(3))
}

pub fn stokeslet_vort(a: Vector3<f64>, r: Vector3<f64>) -> Vector3<f64> {
    let rm = safe_magnitude(r);
    2.0 * a.cross(&r) / rm.powi(3)
}

pub fn stokes_doublet_chwang_u(a: Vector3<f64>, b: Vector3<f64>, r: Vector3<f64>) -> Vector3<f64> {
    let c = a.cross(&b);
    rotlet_chwang_u(c, r) + stresslet_chwang_u(a, b, r)
}

pub fn rotlet_chwang_u(c: Vector3<f64>, r: Vector3<f64>) -> Vector3<f64> {
    let rm = safe_magnitude(r);
    c.cross(&r) / rm.powi(3)
}

pub fn stresslet_chwang_u(a: Vector3<f64>, b: Vector3<f64>, r: Vector3<f64>) -> Vector3<f64> {
    let rm = safe_magnitude(r);
    let term_1 = -a.dot(&b) / rm.powi(3);
    let term_2 = 3.0 * a.dot(&r) * b.dot(&r) / rm.powi(5);
    (term_1 + term_2) * r
}

pub fn potential_doublet_chwang_u(d: Vector3<f64>, r: Vector3<f64>) -> Vector3<f64> {
    let rm = r.magnitude();
    let term_1 = -d / rm.powi(3);
    let term_2 = 3.0 * (d.dot(&r) * r) / rm.powi(5);
    term_1 + term_2
}
