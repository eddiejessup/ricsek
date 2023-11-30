use core::panic;

use log::warn;
use nalgebra::{zero, Matrix3, Vector3};

// r is the vector pointing from the singularity to the point of interest.

fn safe_magnitude(r: Vector3<f64>) -> f64 {
    let rm = r.magnitude();
    if rm < 1e-20 {
        panic!("r is too small: {}", rm);
    } else {
        rm
    }
}

// T_ijk = −6 \hat{r}_i \hat{r}_j \hat{r}_k / |r|^5
pub fn stokeslet_stress_matrix(r: Vector3<f64>) -> [[[f64; 3]; 3]; 3] {
    let coeff = -6.0 / safe_magnitude(r).powi(5);

    let mut result = [[[0.0; 3]; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                result[i][j][k] = coeff * r[i] * r[j] * r[k];
            }
        }
    }
    result
}

// G_ij = δ_ij/|r| + r_i r_j/|r|^3
pub fn stokeslet_matrix(r: Vector3<f64>) -> Matrix3<f64> {
    let rm = safe_magnitude(r); // |r|
    let rm_inv = 1.0 / rm; // 1/|r|
    let rm_inv3 = rm_inv.powi(3); // 1/|r|^3

    // Compute the individual components of the matrix
    let mut g = Matrix3::zeros(); // Initialize a 3x3 zero matrix
    for i in 0..3 {
        for j in 0..3 {
            if i == j {
                g[(i, j)] = rm_inv;
            }
            g[(i, j)] += r[i] * r[j] * rm_inv3;
        }
    }
    g
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

pub fn stokes_doublet_chwang_vort(
    _a: Vector3<f64>,
    _b: Vector3<f64>,
    _r: Vector3<f64>,
) -> Vector3<f64> {
    warn!("Not implemented"); // HACK
    zero()
}

pub fn rotlet_chwang_u(c: Vector3<f64>, r: Vector3<f64>) -> Vector3<f64> {
    let rm = safe_magnitude(r);
    c.cross(&r) / rm.powi(3)
}

pub fn rotlet_chwang_vort(_c: Vector3<f64>, _r: Vector3<f64>) -> Vector3<f64> {
    warn!("Not implemented"); // HACK
    zero()
}

pub fn stresslet_chwang_u(a: Vector3<f64>, b: Vector3<f64>, r: Vector3<f64>) -> Vector3<f64> {
    let rm = safe_magnitude(r);
    let term_1 = -a.dot(&b) / rm.powi(3);
    let term_2 = 3.0 * a.dot(&r) * b.dot(&r) / rm.powi(5);
    (term_1 + term_2) * r
}

pub fn stresslet_chwang_vort(_a: Vector3<f64>, _b: Vector3<f64>, _r: Vector3<f64>) -> Vector3<f64> {
    warn!("Not implemented"); //
    zero()
}

pub fn potential_doublet_chwang_u(d: Vector3<f64>, r: Vector3<f64>) -> Vector3<f64> {
    let rm = r.magnitude();
    let term_1 = -d / rm.powi(3);
    let term_2 = 3.0 * (d.dot(&r) * r) / rm.powi(5);
    term_1 + term_2
}

pub fn potential_doublet_chwang_vort(_d: Vector3<f64>, _r: Vector3<f64>) -> Vector3<f64> {
    warn!("Not implemented"); //
    zero()
}
