use geo::MapCoordsInPlace;
use rand::distributions::Distribution;
use rand::rngs::ThreadRng;

pub fn rotate_point_inplace(v: &mut geo::Point, theta: f64) {
    v.map_coords_in_place(|geo::Coord { x, y }| geo::Coord {
        x: x * theta.cos() - y * theta.sin(),
        y: x * theta.sin() + y * theta.cos(),
    })
}

pub fn random_point<T>(rng: &mut ThreadRng, distr: T) -> geo::Point<f64>
where
    T: Distribution<f64>,
{
    (geo::coord! {x: distr.sample(rng), y: distr.sample(rng)}).into()
}

pub fn array_angle_to_x(v: geo::Coord) -> f64 {
    v.y.atan2(v.x)
}
