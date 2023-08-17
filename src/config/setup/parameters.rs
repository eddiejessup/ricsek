use self::{simulation::SimParams, physical::PhysicalParams};

pub mod physical;
pub mod simulation;
pub mod common;
pub mod singularities;

#[derive(serde::Serialize, serde::Deserialize)]
#[serde(tag = "type")]
pub enum ParametersYaml {
    Physical(PhysicalParams),
    Simulation(SimParams),
}
