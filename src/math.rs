use geo::Point;

pub mod capsule;
pub mod point;

pub fn linspace(start: f64, stop: f64, n: usize) -> Vec<f64> {
    let step = (stop - start) / (n - 1) as f64;
    (0..n).map(|i| start + i as f64 * step).collect()
}

pub fn linspace_grid(start: f64, stop: f64, n: usize) -> Vec<Point> {
    let xs = linspace(start, stop, n);
    xs.iter()
        .flat_map(|x| xs.iter().map(move |y| Point::new(*x, *y)))
        .collect()
}
