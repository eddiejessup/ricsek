use self::{physical::PhysicalParams, simulation::SimParams};

pub mod common;
pub mod physical;
pub mod simulation;
pub mod singularities;

#[derive(serde::Serialize, serde::Deserialize)]
#[serde(tag = "type")]
pub enum ParametersYaml {
    Physical(PhysicalParams),
    Simulation(SimParams),
}
