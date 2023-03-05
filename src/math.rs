use std::ops::Sub;

use ndarray::prelude::*;
use ndarray::Zip;

pub fn norm_sq_one_vec(v: &Array1<f64>) -> f64 {
    v.dot(v)
}

pub fn norm_one_vec(v: &Array1<f64>) -> f64 {
    norm_sq_one_vec(v).sqrt()
}

pub fn rotate_2d_vec_inplace(v: &mut ArrayViewMut1<f64>, theta: f64) {
    let v0 = *v.get(0).unwrap();
    let v1 = *v.get(1).unwrap();
    *(v.get_mut(0).unwrap()) = v0 * theta.cos() - v1 * theta.sin();
    *(v.get_mut(1).unwrap()) = v0 * theta.sin() + v1 * theta.cos();
}

pub fn rotate_2d_vecs_inplace(v: &mut ArrayViewMut2<f64>, theta: ArrayView1<f64>) {
    Zip::from(v.rows_mut())
        .and(theta)
        .for_each(|mut x, theta| rotate_2d_vec_inplace(&mut x, *theta));
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub fn asarray(&self) -> Array1<f64> {
        array![self.x, self.y]
    }
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct LineSegment {
    pub p1: Point,
    pub p2: Point,
}

pub fn array_angle_to_x(v: ArrayView1<f64>) -> f64 {
    v[1].atan2(v[0])
}

impl LineSegment {
    pub fn centre(&self) -> Point {
        let arr = (self.p1.asarray() + self.p2.asarray()) * 0.5;
        Point {
            x: arr[0],
            y: arr[1],
        }
    }

    pub fn as_vector(&self) -> Array1<f64> {
        self.p2.asarray() - self.p1.asarray()
    }

    pub fn angle_to_x(&self) -> f64 {
        let v = self.as_vector();
        array_angle_to_x(v.view())
    }

    pub fn length(&self) -> f64 {
        norm_one_vec(&self.as_vector())
    }

    pub fn nearest_point(&self, p: ArrayView1<f64>) -> Array1<f64> {
        let ss = self.p1.asarray();
        // Get vector from start to end of segment
        let dr = self.p2.asarray() - &ss;
        // Get vector from start of segment to point
        let ss_p = p.sub(&ss);
        // Get the component of ss_p along dr, which is basically the distance we
        // should go along the segment to get to the closest point.
        let nx_line = dr.dot(&ss_p) / norm_sq_one_vec(&dr);
        // Clamp the distance to the segment so we don't go past the end points of
        // the segment.
        let nx_seg = nx_line.min(1.0).max(0.0);
        // Return the point on the segment.
        ss + (dr * nx_seg)
    }
}
