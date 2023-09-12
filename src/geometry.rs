use nalgebra::{Point2, Point3, Vector2, Vector3};

pub mod capsule;
pub mod point;
pub mod line_segment;
pub mod sphere;
pub mod closest;

pub fn arange(start: f64, stop: f64, step: f64) -> Vec<f64> {
    let n = ((stop - start) / step).ceil() as usize;
    (0..n).map(|i| start + i as f64 * step).collect()
}

pub fn linspace(start: f64, stop: f64, n: usize) -> Vec<f64> {
    let step = (stop - start) / (n - 1) as f64;
    (0..n).map(|i| start + i as f64 * step).collect()
}

pub fn grid_2d(l: Vector2<f64>, step: f64) -> Vec<Point2<f64>> {
    let l_half = l / 2.0;

    let mut samples: Vec<Point2<f64>> = Vec::new();
    for x in arange(-l_half.x, l_half.x, step) {
        for y in arange(-l_half.y, l_half.y, step) {
            samples.push(Point2::new(x, y));
        }
    }
    samples
}

pub fn grid(l: Vector3<f64>, n: usize) -> Vec<Point3<f64>> {
    // step ^ 3 * n-arrows = lx * ly * lz
    // step = cube-root(lx * ly * lz / n-arrows)
    let step = (l.x * l.y * l.z / n as f64).powf(1.0 / 3.0);
    let l_half = l / 2.0;

    let mut samples: Vec<Point3<f64>> = Vec::new();
    for z in arange(-l_half.z, l_half.z, step) {
        for p in grid_2d(Vector2::new(l.x, l.y), step) {
            samples.push(Point3::new(p.x, p.y, z));
        }
    }
    samples
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
