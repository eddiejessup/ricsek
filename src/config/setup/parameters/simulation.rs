use super::common::BoundaryConfig;

#[derive(serde::Serialize, serde::Deserialize, Debug)]
pub struct SimParams {
    // Time step.
    pub dt: f64,
    // System.
    pub boundaries: BoundaryConfig,
    pub singularities: Vec<super::singularities::Singularity>,
    // Agent shape.
    pub agent_radius: f64,
    pub agent_aspect_ratio: f64,
    // Agent propulsion.
    pub agent_propulsion_speed: f64,
    // Agent thermal noise.
    pub agent_translational_diffusion_coefficient: f64,
    pub agent_rotational_diffusion_coefficient: f64,
    // Agent-agent hydrodynamics.
    pub agent_agent_hydro_a: f64,
    pub agent_agent_hydro_b: f64,
    // Electrostatic repulsion, used for both agent-agent and agent-surface
    // interactions.
    pub agent_object_hertz_velocity_coefficient: f64,
    // Agent-surface hydrodynamics.
    // Coefficient determining strength of translational and rotational velocity
    // induced by agent-surface interactions.
    pub agent_obstacle_hydro_strength: f64,
}

impl SimParams {
  pub fn to_steps(&self, t: f64) -> usize {
      (t / self.dt).ceil() as usize
  }
}
