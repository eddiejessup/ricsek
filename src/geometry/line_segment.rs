use nalgebra::{Point3, Rotation3, UnitVector3, Vector3};

use super::closest::Closest;

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct LineSegment {
    pub start: Point3<f64>,
    pub end: Point3<f64>,
}

#[derive(Debug, Clone)]
pub struct BoundingBox {
    pub min: Point3<f64>,
    pub max: Point3<f64>,
}

impl BoundingBox {
    pub fn overlaps(&self, b: BoundingBox) -> bool {
        self.min.x <= b.max.x
            && self.max.x >= b.min.x
            && self.min.y <= b.max.y
            && self.max.y >= b.min.y
            && self.min.z <= b.max.z
            && self.max.z >= b.min.z
    }

    pub fn union(&self, b: &BoundingBox) -> BoundingBox {
        BoundingBox {
            min: Point3::new(
                self.min.x.min(b.min.x),
                self.min.y.min(b.min.y),
                self.min.z.min(b.min.z),
            ),
            max: Point3::new(
                self.max.x.max(b.max.x),
                self.max.y.max(b.max.y),
                self.max.z.max(b.max.z),
            ),
        }
    }
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

    pub fn approx_closest_points_on_segment(
        &self,
        p: &LineSegment,
        n: usize,
    ) -> (Point3<f64>, Point3<f64>) {
        // Generate 'n' equally spaced points from self.start to self.end.
        // Generate 'n' equally spaced points from p.start to p.end.
        // Compute pairwise distances.
        // Return the pair of points with the smallest distance.
        let mut min_dist = std::f64::MAX;
        let mut min_pair = (self.start, p.start);
        let start_end = self.start_end();
        let p_start_end = p.start_end();
        let step = 1.0 / (n as f64);
        for i in 0..(n + 1) {
            let t = (i as f64) * step;
            let self_point = self.start + start_end.scale(t);
            for j in 0..(n + 1) {
                let u = (j as f64) * step;
                let p_point = p.start + p_start_end.scale(u);
                let dist = nalgebra::distance(&self_point, &p_point);
                if dist < min_dist {
                    min_dist = dist;
                    min_pair = (self_point, p_point);
                }
            }
        }
        min_pair
    }

    pub fn bounding_box(&self) -> BoundingBox {
        BoundingBox {
            min: Point3::new(
                self.start.x.min(self.end.x),
                self.start.y.min(self.end.y),
                self.start.z.min(self.end.z),
            ),
            max: Point3::new(
                self.start.x.max(self.end.x),
                self.start.y.max(self.end.y),
                self.start.z.max(self.end.z),
            ),
        }
    }
}

impl Closest for LineSegment {
    fn centroid(&self) -> Point3<f64> {
        self.centroid()
    }

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
