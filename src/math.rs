use derive_more::{Add, Mul, Neg, Sub};
use geo::{MapCoords, MapCoordsInPlace};
use rand::distributions::Distribution;
use rand::rngs::ThreadRng;

pub mod capsule;

#[derive(Debug, Add, Neg, Sub, Mul)]
pub struct Vector(pub geo::Coord<f64>);

pub fn rotate_point(v: geo::Point, theta: f64) -> geo::Point {
    v.map_coords(|c| rotate_coords(c, theta))
}

pub fn unit_angle_cos_sin(u: geo::Point, v: geo::Point) -> (f64, f64) {
    let dot_product = u.x() * v.x() + u.y() * v.y();
    let cross_product = u.x() * v.y() - u.y() * v.x();
    let theta = cross_product.atan2(dot_product);
    (theta.cos(), theta.sin())
}

pub fn rotate_point_inplace(v: &mut geo::Point, theta: f64) {
    v.map_coords_in_place(|c| rotate_coords(c, theta));
}

pub fn rotate_coords(c: geo::Coord, theta: f64) -> geo::Coord {
    geo::Coord {
        x: c.x * theta.cos() - c.y * theta.sin(),
        y: c.x * theta.sin() + c.y * theta.cos(),
    }
}

pub fn random_coord<T>(rng: &mut ThreadRng, distr: T) -> geo::Coord
where
    T: Distribution<f64>,
{
    geo::coord! {x: distr.sample(rng), y: distr.sample(rng)}
}

pub fn array_angle_to_x(v: geo::Coord) -> f64 {
    v.y.atan2(v.x)
}

// If we assume d << r, then
// theta = acos(1 - d/r) ~ d/r
// so we get (d/r) * r / pi = d / pi
pub fn circle_overlap_circumference(r: f64, d: f64) -> f64 {
    let theta = (1.0 - (d / r)).acos();
    (theta / std::f64::consts::PI) * r
}
