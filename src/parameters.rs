use crate::math::capsule::Capsule;

pub mod physical;
pub mod simulation;

pub struct SimSetup {
    pub params: simulation::SimParams,
    pub capsules: Vec<Capsule>,
}

pub struct RunParams {
    pub t_max: f64,
    pub dstep_view: usize,
    pub run_id: usize,
}
