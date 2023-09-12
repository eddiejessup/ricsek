use nalgebra::{Point3, Rotation3, UnitVector3, Vector3};

use super::closest::Closest;

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct LineSegment {
    pub start: Point3<f64>,
    pub end: Point3<f64>,
}

impl LineSegment {
    pub fn new(r: Point3<f64>, d: Vector3<f64>) -> Self {
        let (start, end) = Self::rs_from_centre(r, d);
        LineSegment { start, end }
    }

    pub fn rs_from_centre(r: Point3<f64>, d: Vector3<f64>) -> (Point3<f64>, Point3<f64>) {
        let rd_half = d.scale(0.5);
        (r - rd_half, r + rd_half)
    }

    pub fn translate(&mut self, dr: Vector3<f64>) {
        self.start += dr;
        self.end += dr;
    }

    pub fn rotate(&mut self, rot: Rotation3<f64>) {
        let (start, end) = Self::rs_from_centre(self.centroid(), rot * self.start_end());
        self.start = start;
        self.end = end;
    }

    pub fn centroid(&self) -> Point3<f64> {
        nalgebra::center(&self.start, &self.end)
    }

    pub fn start_end(&self) -> Vector3<f64> {
        self.end - self.start
    }

    pub fn length(&self) -> f64 {
        self.start_end().norm()
    }

    pub fn u_start_end(&self) -> UnitVector3<f64> {
        UnitVector3::new_normalize(self.start_end())
    }

    pub fn closest_point(&self, p: Point3<f64>) -> Point3<f64> {
        let u_start_end = self.u_start_end();
        let t = u_start_end.dot(&(p - self.start));
        if t < 0.0 {
            self.start
        } else if t > self.length() {
            self.end
        } else {
            self.start + u_start_end.scale(t)
        }
    }

    pub fn closest_point_on_line_segment(&self, other: &LineSegment) -> (Point3<f64>, Point3<f64>) {
        let (a, b, c, d) = (self.start, self.end, other.start, other.end);

        let ab = b - a;
        let cd = d - c;
        let ac = c - a;

        let e = ab.dot(&ab);
        let f = ab.dot(&cd);
        let g = cd.dot(&cd);
        let h = ab.dot(&ac);
        let i = cd.dot(&ac);

        let denominator = f * f - g * e;

        // Check if lines are almost parallel
        let (t, u) = if denominator.abs() > 1e-28 {
            // println!("Lines are not almost parallel, denominator={}", denominator);
            ((g * h - f * i) / denominator, (e * i - f * h) / denominator)
        } else {
            // println!("Lines are almost parallel, denominator={}", denominator);
            (0.0, -h / f)
        };

        let t_clip = t.max(0.0).min(1.0);
        let u_clip = u.max(0.0).min(1.0);

        let p = a + ab * t_clip;
        let q = c + cd * u_clip;
        (p, q)
    }
}

impl Closest for LineSegment {
    // Along each axis, find the point on the line segment
    // furthest along that axis in the +/- direction.
    fn furthest_point_along_axis(&self, i: usize, positive: bool) -> Point3<f64> {
        if self.end[i] > self.start[i] {
            if positive {
                self.end
            } else {
                self.start
            }
        } else {
            if positive {
                self.start
            } else {
                self.end
            }
        }
    }
}
