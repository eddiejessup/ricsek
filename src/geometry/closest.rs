use nalgebra::Point3;

pub trait Closest {
    fn furthest_point_along_axis(&self, i: usize, positive: bool) -> Point3<f64>;
}
