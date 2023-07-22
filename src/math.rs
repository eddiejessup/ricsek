use nalgebra::{Vector2, Point2};

pub mod capsule;
pub mod point;

pub fn linspace(start: f64, stop: f64, n: usize) -> Vec<f64> {
    let step = (stop - start) / (n - 1) as f64;
    (0..n).map(|i| start + i as f64 * step).collect()
}

pub fn linspace_grid(start: f64, stop: f64, n: usize) -> Vec<Point2<f64>> {
    let xs = linspace(start, stop, n);
    xs.iter()
        .flat_map(|x| xs.iter().map(move |y| Point2::new(*x, *y)))
        .collect()
}

pub fn angle_to_x(v: &Vector2<f64>) -> f64 {
    let angle = v.angle(&Vector2::x());
    if v.y < 0.0 {
        2.0 * std::f64::consts::PI - angle
    } else {
      angle

    }
}
