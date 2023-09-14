use nalgebra::Point3;

pub trait Closest {
    fn centroid(&self) -> Point3<f64>;
    fn furthest_point_along_axis(&self, i: usize, positive: bool) -> Point3<f64>;
}
