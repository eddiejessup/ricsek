use ndarray::prelude::*;

pub mod params;

pub struct SimState {
    pub u_p: Array2<f64>,
    pub r: Array2<f64>,
    pub t: f64,
    pub step: usize,
}

impl SimState {
    pub fn new(u_p: Array2<f64>, r: Array2<f64>) -> SimState {
        SimState {
            u_p,
            r,
            t: 0.0,
            step: 0,
        }
    }
}
