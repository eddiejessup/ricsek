use geo::{coord, Centroid, EuclideanLength, Coord};
use nalgebra::{vector, Point2, Unit, UnitVector2, Vector2};

use super::angle_to_x;

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Capsule<T = f64>
where
    T: geo::CoordNum,
{
    segment: geo::Line<T>,
    pub radius: T,
}

fn coord_to_point2(c: Coord) -> Point2<f64> {
    Point2::new(c.x, c.y)
}

impl Capsule {
    pub fn new(start: Point2<f64>, end: Point2<f64>, radius: f64) -> Capsule {
        Capsule {
            segment: geo::Line {
                start: coord! {x: start.x, y: start.y},
                end: coord! {x: end.x, y: end.y},
            },
            radius,
        }
    }

    pub fn centroid(&self) -> Point2<f64> {
        coord_to_point2(self.segment.centroid().0)
    }

    pub fn start_point(&self) -> Point2<f64> {
        coord_to_point2(self.segment.start)
    }

    pub fn end_point(&self) -> Point2<f64> {
        coord_to_point2(self.segment.start)
    }

    pub fn length(&self) -> f64 {
        self.segment_length() + 2.0 * self.radius
    }

    pub fn width(&self) -> f64 {
        2.0 * self.radius
    }

    pub fn segment_length(&self) -> f64 {
        self.segment.euclidean_length()
    }

    pub fn angle_to_x(&self) -> f64 {
        let r = self.segment.end - self.segment.start;
        angle_to_x(&vector!(r.x, r.y))
    }

    pub fn closest_point(
        &self,
        p: Point2<f64>,
    ) -> (Point2<f64>, Vector2<f64>, f64, UnitVector2<f64>) {
        let c = match geo::ClosestPoint::closest_point(&self.segment, &geo::point!(x: p.x, y: p.y))
        {
            geo::Closest::Intersection(_p) => {
                panic!("Intersection: point lies on capsule centerline");
            }
            geo::Closest::Indeterminate => {
                panic!("No single closest point to capsule centerline");
            }
            geo::Closest::SinglePoint(p) => Point2::new(p.x(), p.y()),
        };

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
}
