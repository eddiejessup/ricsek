#[derive(serde::Serialize, serde::Deserialize, Debug)]
pub struct SimParams {
    pub dt: f64,
    pub l: f64,
    // Agent shape.
    pub agent_radius: f64,
    pub agent_aspect_ratio: f64,
    // Agent propulsion.
    pub agent_propulsion_speed: f64,
    // Agent thermal noise.
    pub agent_translational_diffusion_coefficient: f64,
    pub agent_rotational_diffusion_coefficient: f64,
    // Agent-agent hydrodynamics.
    pub agent_stresslet_force_longitudinal: f64,
    pub agent_stresslet_force_transverse: f64,
    pub agent_stresslet_force_rotational: f64,
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

    pub fn l_half(&self) -> f64 {
        self.l * 0.5
    }
}
