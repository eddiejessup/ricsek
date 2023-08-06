use nalgebra::{Point3, UnitVector2, UnitVector3, Vector3};
use rand::distributions::Distribution;
use rand::rngs::ThreadRng;
use rand_distr::StandardNormal;

pub fn unit_angle_cos_sin_2d(u: UnitVector2<f64>, v: UnitVector2<f64>) -> (f64, f64) {
    let dot_product = u.dot(&v);
    let cross_product = u.perp(&v);
    let theta = cross_product.atan2(dot_product);
    (theta.cos(), theta.sin())
}

pub fn random_vector<T>(rng: &mut ThreadRng, distr: T) -> Vector3<f64>
where
    T: Distribution<f64>,
{
    Vector3::new(distr.sample(rng), distr.sample(rng), distr.sample(rng))
}

pub fn random_point<T>(rng: &mut ThreadRng, distr: T) -> Point3<f64>
where
    T: Distribution<f64>,
{
    Point3::from(random_vector(rng, distr))
}

// Expects distribution to be a normal distribution with mean 0 and variance 1.
pub fn random_unit_vector(rng: &mut ThreadRng) -> UnitVector3<f64> {
    nalgebra::Unit::new_normalize(random_vector(rng, StandardNormal))
}
