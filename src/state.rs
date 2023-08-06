use nalgebra::{Point3, UnitVector3};

pub struct SimState {
    pub agents: Vec<Agent>,
    pub t: f64,
    pub step: usize,
}

pub struct Agent {
    pub r: Point3<f64>,
    pub u: UnitVector3<f64>,
}

impl SimState {
    pub fn new(agents: Vec<Agent>) -> SimState {
        SimState {
            agents,
            t: 0.0,
            step: 0,
        }
    }
}
