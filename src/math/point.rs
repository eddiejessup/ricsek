use geo::{MapCoords, MapCoordsInPlace};
use rand::distributions::Distribution;
use rand::rngs::ThreadRng;
use std::iter::Sum;

pub struct Point {
    pub x: f64,
    pub y: f64,
}

pub fn coord_magnitude(c: geo::Coord) -> f64 {
    (c.x.powi(2) + c.y.powi(2)).sqrt()
}

pub fn point_magnitude(c: geo::Point) -> f64 {
    coord_magnitude(c.0)
}

pub fn rotate_point(v: geo::Point, theta: f64) -> geo::Point {
    v.map_coords(|c| rotate_coords(c, theta))
}

pub fn cross(u: geo::Point, v: geo::Point) -> f64 {
    u.x() * v.y() - u.y() * v.x()
}

pub fn unit_angle_cos_sin(u: geo::Point, v: geo::Point) -> (f64, f64) {
    let dot_product = u.x() * v.x() + u.y() * v.y();
    let cross_product = cross(u, v);
    let theta = cross_product.atan2(dot_product);
    (theta.cos(), theta.sin())
}

pub fn point_unit(u: geo::Point) -> geo::Point {
    u / point_magnitude(u)
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

pub struct PointSum(pub geo::Point);

impl Sum<PointSum> for PointSum {
    fn sum<I>(iter: I) -> Self
    where
        I: Iterator<Item = PointSum>,
    {
        let mut sum = geo::Point::new(0.0, 0.0);
        for p in iter {
            sum += p.0;
        }
        PointSum(sum)
    }
}
