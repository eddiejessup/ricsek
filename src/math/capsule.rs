use geo::{Centroid, EuclideanLength};

use super::point;

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Capsule<T = f64>
where
    T: geo::CoordNum,
{
    pub segment: geo::Line<T>,
    pub radius: T,
}

impl Capsule {
    pub fn new(start: geo::Coord, end: geo::Coord, radius: f64) -> Capsule {
        Capsule {
            segment: geo::Line { start, end },
            radius,
        }
    }

    pub fn centroid(&self) -> geo::Point {
        self.segment.centroid()
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
        point::array_angle_to_x(self.segment.end - self.segment.start)
    }

    pub fn closest_point(&self, p: geo::Point) -> (geo::Point, geo::Point, f64, geo::Point) {
        let c = match geo::ClosestPoint::closest_point(&self.segment, &p) {
            geo::Closest::Intersection(_p) => {
                panic!("Intersection: point lies on capsule centerline");
            }
            geo::Closest::Indeterminate => {
                panic!("No single closest point to capsule centerline");
            }
            geo::Closest::SinglePoint(p) => p,
        };

        // The vector from the point to the centerline.
        let p_c = c - p;
        // The distance from the point to the centerline.
        let p_c_mag = point::point_magnitude(p_c);
        // Unit vector from the point towards the centerline.
        let p_c_u = p_c / p_c_mag;
        // The point on the surface of the capsule.
        let s = c - (p_c_u * self.radius);

        let dist_to_surface = p_c_mag - self.radius;

        let to_surface_vec = s - p;

        let centrepoint_unit = p_c / p_c_mag;

        (s, to_surface_vec, dist_to_surface, centrepoint_unit)
    }

    pub fn layout_to_capsules(c: Vec<geo::Line>, radius: f64, l: f64) -> Vec<Capsule> {
        c.iter()
            .map(|s| Capsule::new(s.start * l / 2.0, s.end * l / 2.0, radius))
            .collect()
    }
}
