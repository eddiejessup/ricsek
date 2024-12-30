use crate::{
    config::setup::parameters::singularities::SingularityParams,
    geometry::{line_segment::LineSegment, point::ObjectPoint},
};
use log::debug;
use nalgebra::{Point3, UnitVector3, Vector3};
use rand_pcg::Pcg64Mcg;

#[derive(Clone)]
pub struct SimState {
    pub agents: Vec<Agent>,
    pub t: f64,
    pub step: usize,
    pub rng: Pcg64Mcg,
}

impl SimState {
    pub fn new(agents: Vec<Agent>, rng: Pcg64Mcg) -> SimState {
        SimState {
            agents,
            t: 0.0,
            step: 0,
            rng,
        }
    }
}

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct Agent {
    pub seg: LineSegment,
    pub tail_phase: f64,
}

impl Agent {
    pub fn new(r: Point3<f64>, d: Vector3<f64>, tail_phase: f64) -> Self {
        debug!("Agent::new(r={}, d={})", 1e6 * r, 1e6 * d);
        Agent {
            seg: LineSegment::new(r, d),
            tail_phase,
        }
    }

    pub fn r1(&self) -> Point3<f64> {
        self.seg.start
    }

    pub fn r2(&self) -> Point3<f64> {
        self.seg.end
    }

    pub fn u(&self) -> UnitVector3<f64> {
        self.seg.u_start_end()
    }

    pub fn r1_stretch_force(&self, d0: f64, k: f64) -> Vector3<f64> {
        // If d is large, we want the spring to contract,
        // which implies a force from r1 to r2.
        // d points from r1 to r2, so we want a force along d.
        // This means the coefficient we scale d by should be positive.
        let d = self.seg.start_end();
        let displacement = d.norm() - d0;
        debug!(
            "d0={}, d={}, displacement={}",
            1e6 * d0,
            1e6 * d.norm(),
            1e6 * displacement
        );
        d.normalize().scale(k * displacement)
    }
}

// A step summary is a summary of what happened or was computed during a single step.

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct AgentStepSummary {
    pub f_agent_electro: Vector3<f64>,
    pub torque_agent_electro: Vector3<f64>,
    pub f_propulsion: Vector3<f64>,
    pub f_boundary_electro: Vector3<f64>,
    pub torque_boundary_electro: Vector3<f64>,
    pub v_fluid: Vector3<f64>,
}

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct StepSummary {
    pub agent_summaries: Vec<AgentStepSummary>,
    pub singularities: Vec<(ObjectPoint, SingularityParams)>,
    pub fluid_flow: Vec<Vector3<f64>>,
}

#[derive(Clone)]
pub struct SimStateWithSummary {
    pub sim_state: SimState,
    // Optional because we might not compute a summary, especially for the initial state.
    pub step_summary: Option<StepSummary>,
}
