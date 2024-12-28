use nalgebra::{Point3, UnitVector2, UnitVector3, Vector3};
use rand::distributions::Distribution;
use rand_distr::StandardNormal;

#[derive(Clone)]
pub struct ObjectPoint {
    pub object_id: u32,
    pub position: Point3<f64>,
    pub position_com: Point3<f64>,
}

impl ObjectPoint {
    pub fn point_object(object_id: u32, position: Point3<f64>) -> Self {
        Self {
            object_id,
            position,
            position_com: position,
        }
    }
}

pub fn unit_angle_cos_sin_2d(u: UnitVector2<f64>, v: UnitVector2<f64>) -> (f64, f64) {
    let dot_product = u.dot(&v);
    let cross_product = u.perp(&v);
    let theta = cross_product.atan2(dot_product);
    (theta.cos(), theta.sin())
}

pub fn random_vector<R: rand::Rng, T: Distribution<f64>>(rng: &mut R, distr: T) -> Vector3<f64> {
    Vector3::new(distr.sample(rng), distr.sample(rng), distr.sample(rng))
}

pub fn random_point<R: rand::Rng, T: Distribution<f64>>(rng: &mut R, distr: T) -> Point3<f64> {
    Point3::from(random_vector(rng, distr))
}

// Expects distribution to be a normal distribution with mean 0 and variance 1.
pub fn random_unit_vector<R: rand::Rng>(rng: &mut R) -> UnitVector3<f64> {
    nalgebra::Unit::new_normalize(random_vector(rng, StandardNormal))
}
