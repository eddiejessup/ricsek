use self::{simulation::SimParams, physical::PhysicalParams};

pub mod physical;
pub mod simulation;

#[derive(serde::Serialize, serde::Deserialize, Debug)]
pub struct Parameters {
    pub sim_params: SimParams,
    pub physical_params: Option<PhysicalParams>,
}
