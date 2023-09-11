use nalgebra::{Point3, UnitVector3, Vector3};

#[derive(serde::Serialize, serde::Deserialize)]
pub struct AgentStepSummary {
    pub f_agent_electro: Vector3<f64>,
    pub f_agent_hydro: Vector3<f64>,
    pub f_propulsion: Vector3<f64>,
    pub f_boundary_electro: Vector3<f64>,
    pub f_singularity: Vector3<f64>,
}

pub struct SimState {
    pub agents: Vec<Agent>,
    pub t: f64,
    pub step: usize,
    pub summary: Option<Vec<AgentStepSummary>>,
}

#[derive(serde::Serialize, serde::Deserialize, Clone)]
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
            summary: None,
        }
    }
}
