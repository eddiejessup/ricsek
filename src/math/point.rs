use nalgebra::{Point2, UnitVector2, Vector2};
use rand::distributions::Distribution;
use rand::rngs::ThreadRng;

pub fn unit_angle_cos_sin(u: UnitVector2<f64>, v: UnitVector2<f64>) -> (f64, f64) {
    let dot_product = u.dot(&v);
    let cross_product = u.perp(&v);
    let theta = cross_product.atan2(dot_product);
    (theta.cos(), theta.sin())
}

pub fn random_vector<T>(rng: &mut ThreadRng, distr: T) -> Vector2<f64>
where
    T: Distribution<f64>,
{
    Vector2::new(distr.sample(rng), distr.sample(rng))
}

pub fn random_point<T>(rng: &mut ThreadRng, distr: T) -> Point2<f64>
where
    T: Distribution<f64>,
{
    Point2::from(random_vector(rng, distr))
}
