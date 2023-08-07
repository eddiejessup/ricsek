use nalgebra::{Vector3, Point3};

fn wrap1(x: f64, l: f64) -> f64 {
    if x < -l * 0.5 {
        // let n_wrap = (x / (l * 0.5)).abs().ceil();
        // if n_wrap > 1.0 {
        //   panic!("n_wrap = {} > 1", n_wrap);
        // }
        x + l
    } else if x > l * 0.5 {
        x - l
    } else {
        x
    }
}

pub fn wrap(r: &mut Point3<f64>, l: Vector3<f64>) {
    r.x = wrap1(r.x, l.x);
    r.y = wrap1(r.y, l.y);
    r.z = wrap1(r.z, l.z);
}
