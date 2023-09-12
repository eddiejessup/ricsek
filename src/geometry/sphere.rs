use nalgebra::{Point3, Unit, UnitVector3, Vector3};

use super::closest::Closest;

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Sphere {
    pub point: Point3<f64>,
    pub radius: f64,
}

impl Sphere {
    pub fn closest_point(
        &self,
        p: Point3<f64>,
    ) -> (Point3<f64>, Vector3<f64>, f64, UnitVector3<f64>) {
        // The vector from the point to the sphere centre.
        let p_c = self.point - p;
        // The distance from the point to the sphere centre.
        let p_c_mag = p_c.magnitude();
        // Unit vector from the point towards the sphere centre.
        let p_c_u = Unit::new_normalize(p_c);
        // The point on the surface of the sphere.
        let s = self.point - p_c_u.scale(self.radius);

        let dist_to_surface = p_c_mag - self.radius;

        let to_surface_vec = s - p;

        (s, to_surface_vec, dist_to_surface, p_c_u)
    }
}

impl Closest for Sphere {
    fn furthest_point_along_axis(&self, i: usize, positive: bool) -> Point3<f64> {
        self.point + Vector3::ith(i, self.radius * if positive { 1.0 } else { -1.0 })
    }
}
