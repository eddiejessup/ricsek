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
    pub agent_rod_length: f64,
    pub agent_linear_spring_stiffness: f64,
    // Agent propulsion.
    pub agent_propulsion_force: f64,
    // Agent thermal noise.
    pub agent_translational_diffusion_coefficient: f64,
    pub agent_rotational_diffusion_coefficient: f64,
    // Agent-agent hydrodynamics.
    pub agent_agent_hydro_a: f64,
    pub agent_agent_hydro_b: f64,
    // Electrostatic repulsion, used for both agent-agent and agent-surface
    // interactions.
    pub agent_object_hertz_force_coefficient: f64,
    pub agent_mobility: f64,
    // Agent-surface hydrodynamics.
    // Coefficient determining strength of translational and rotational velocity
    // induced by agent-surface interactions.
    pub agent_obstacle_hydro_strength: f64,
}

impl SimParams {
  pub fn to_steps(&self, t: f64) -> usize {
      (t / self.dt).ceil() as usize
  }

  pub fn agent_inter_sphere_length(&self) -> f64 {
      self.agent_rod_length + 2.0 * self.agent_radius
  }

  pub fn agent_propulsion_speed(&self) -> f64 {
      self.agent_propulsion_force * self.agent_mobility
  }
}
