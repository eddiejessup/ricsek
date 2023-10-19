use std::f64::consts::PI;

use nalgebra::{Point3, Unit, UnitVector3, Vector3};

use super::{
    closest::Closest,
    line_segment::{BoundingBox, LineSegment},
};

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Capsule {
    pub segment: LineSegment,
    pub radius: f64,
}

impl Capsule {
    pub fn centroid(&self) -> Point3<f64> {
        self.segment.centroid()
    }

    pub fn segment_length(&self) -> f64 {
        self.segment.length()
    }

    pub fn length(&self) -> f64 {
        self.segment.length() + 2.0 * self.radius
    }

    pub fn width(&self) -> f64 {
        2.0 * self.radius
    }

    pub fn volume(&self) -> f64 {
        // Volume of a cylinder plus two end-half-spheres.
        (PI * self.radius.powi(2) * self.segment_length()) + (4.0 / 3.0) * PI * self.radius.powi(3)
    }

    pub fn area(&self) -> f64 {
        // Area of a cylinder plus two end-caps.
        (2.0 * PI * self.radius * self.segment_length()) + (4.0 * PI * self.radius.powi(2))
    }

    pub fn closest_point(
        &self,
        p: Point3<f64>,
    ) -> (Point3<f64>, Vector3<f64>, f64, UnitVector3<f64>) {
        let c = self.segment.closest_point(p);

        // The vector from the point to the centerline.
        let p_c = c - p;
        // The distance from the point to the centerline.
        let p_c_mag = p_c.magnitude();
        // Unit vector from the point towards the centerline.
        let p_c_u = Unit::new_normalize(p_c);
        // The point on the surface of the capsule.
        let s = c - (p_c_u.into_inner() * self.radius);

        let dist_to_surface = p_c_mag - self.radius;

        let to_surface_vec = s - p;

        (s, to_surface_vec, dist_to_surface, p_c_u)
    }

    pub fn bounding_box(&self) -> BoundingBox {
        capsule_bounding_box(&self.segment, self.radius)
    }
}

pub fn capsule_bounding_box(segment: &LineSegment, radius: f64) -> BoundingBox {
    let bb = segment.bounding_box();
    BoundingBox {
        min: bb.min.coords.add_scalar(-radius).into(),
        max: bb.max.coords.add_scalar(radius).into(),
    }
}

impl Closest for Capsule {
    fn centroid(&self) -> Point3<f64> {
        self.segment.centroid()
    }

    fn furthest_point_along_axis(&self, i: usize, positive: bool) -> Point3<f64> {
        self.segment.furthest_point_along_axis(i, positive)
            + Vector3::ith(i, self.radius * if positive { 1.0 } else { -1.0 })
    }
}
