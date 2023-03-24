pub struct SimState {
    pub agents: Vec<Agent>,
    pub t: f64,
    pub step: usize,
}

pub struct Agent {
    pub r: geo::Point,
    pub u: geo::Point,
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
