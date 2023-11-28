use super::common::BoundaryConfig;

#[derive(serde::Serialize, serde::Deserialize, Debug)]
pub struct SimParams {
    // Time step.
    pub dt: f64,
    // System.
    pub boundaries: BoundaryConfig,
    pub singularities: Vec<super::singularities::Singularity>,
    pub enable_fluid: bool,
    // Agent shape.
    pub agent_radius: f64,
    pub agent_inter_sphere_length: f64,
    // Agent propulsion.
    pub enable_agent_propulsion: bool,
    //   Force acting on the fluid that propels the bacterium forward.
    pub agent_propulsion_force: f64,
    pub agent_translational_mobility: f64,
    pub agent_rotational_mobility: f64,
    //   Strength of the bacterium's force on the fluid as transmitted through
    //   the fluid. i.e. this depends on the fluid's viscosity. Not physically
    //   independent, but can be in simulation terms.
    pub agent_propulsive_stokeslet_strength: f64,
    pub agent_propulsive_rotlet_strength: f64,
    // Agent thermal noise.
    pub agent_translational_diffusion_coefficient: f64,
    pub agent_rotational_diffusion_coefficient: f64,
    // Electrostatic repulsion, used for both agent-agent and agent-surface
    // interactions.
    pub agent_object_hertz_force_coefficient: f64,
    pub enable_agent_agent_electro: bool,
    pub enable_agent_boundary_electro: bool,
}

impl SimParams {
    pub fn to_steps(&self, t: f64) -> usize {
        (t / self.dt).ceil() as usize
    }

    pub fn agent_propulsion_speed(&self) -> f64 {
        self.agent_propulsion_force * self.agent_translational_mobility
    }
}
