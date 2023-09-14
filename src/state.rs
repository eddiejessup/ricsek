use log::debug;
use nalgebra::{Point3, UnitVector3, Vector3};

use crate::geometry::{
    capsule::capsule_bounding_box,
    line_segment::{BoundingBox, LineSegment},
};

#[derive(serde::Serialize, serde::Deserialize)]
pub struct AgentStepSummary {
    pub f_agent_electro: Vector3<f64>,
    pub torque_agent_electro: Vector3<f64>,
    pub f_agent_hydro: Vector3<f64>,
    pub f_propulsion: Vector3<f64>,
    pub f_boundary_electro: Vector3<f64>,
    pub torque_boundary_electro: Vector3<f64>,
    pub f_singularity: Vector3<f64>,
}

pub struct SimState {
    pub agents: Vec<Agent>,
    pub t: f64,
    pub step: usize,
    pub summary: Option<Vec<AgentStepSummary>>,
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

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct Agent {
    pub seg: LineSegment,
    pub th1: f64,
    pub th2: f64,
}

impl Agent {
    pub fn new(r: Point3<f64>, d: Vector3<f64>) -> Self {
        debug!("Agent::new(r={}, d={})", 1e6 * r, 1e6 * d);
        Agent {
            seg: LineSegment::new(r, d),
            th1: 0.0,
            th2: 0.0,
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

    pub fn bounding_box(&self, radius: f64) -> BoundingBox {
        capsule_bounding_box(&self.seg, radius)
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
