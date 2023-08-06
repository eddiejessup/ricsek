use nalgebra::{Rotation3, UnitVector3, Vector3};
use rand::rngs::ThreadRng;
use rand_distr::{Distribution, Normal, Uniform};

pub fn trans_brownian_distr(d_diff: f64, dt: f64) -> Normal<f64> {
    let l = (2.0 * d_diff * dt).sqrt();
    Normal::new(0.0, l).unwrap()
}

pub fn rot_brownian_distr(d_diff: f64, dt: f64) -> Normal<f64> {
    let l = (2.0 * d_diff * dt).sqrt();
    Normal::new(0.0, l).unwrap()
}

pub fn random_rotation(
    rng: &mut ThreadRng,
    uniform_theta: &Uniform<f64>,
    uniform_cos_phi: &Uniform<f64>,
    normal_distribution: &Normal<f64>,
) -> Rotation3<f64> {
    let theta = uniform_theta.sample(rng);
    let cos_phi = uniform_cos_phi.sample(rng);
    let sin_phi = (1.0 - cos_phi * cos_phi).sqrt();
    let x = sin_phi * theta.cos();
    let y = sin_phi * theta.sin();
    let z = cos_phi;
    let direction = UnitVector3::new_normalize(Vector3::new(x, y, z));

    let angle = normal_distribution.sample(rng);

    Rotation3::from_axis_angle(&direction, angle)
}
