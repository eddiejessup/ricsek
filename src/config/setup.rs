pub mod agents;
pub mod parameters;

use std::{error::Error, fs::File, io::Read, path::Path};

use self::{agents::AgentInitializationConfig, parameters::simulation::SimParams};

#[derive(serde::Serialize, serde::Deserialize)]
struct SetupConfigYaml {
    parameters: parameters::ParametersYaml,
    agent_initialization: Vec<AgentInitializationConfig>,
}

pub struct SetupConfig {
    pub parameters: SimParams,
    pub agent_initialization: Vec<AgentInitializationConfig>,
}

impl SetupConfig {
    pub fn parse<P: AsRef<Path>>(path: P) -> Result<Self, Box<dyn Error>> {
        let mut file = File::open(path)?;
        let mut contents = String::new();
        file.read_to_string(&mut contents)?;
        let config_raw: SetupConfigYaml = serde_yaml::from_str(&contents)?;
        let parameters = match config_raw.parameters {
            parameters::ParametersYaml::Physical(physical_params) => physical_params.as_params(),
            parameters::ParametersYaml::Simulation(sim_params) => sim_params,
        };
        let config = SetupConfig {
            parameters,
            agent_initialization: config_raw.agent_initialization,
        };
        Ok(config)
    }

    pub fn print(&self) {
        println!(
            "\
Simulation parameters:
  Environment:
    Timestep: {dt} s
    System length (µm): {l}

  Agents:
    Effective radius: {ag_radius} µm
    Translational diffusion rate: {d_trans_diff:.1} µm^2/s
    Rotational diffusion rate: {d_rot_diff:.1} rad^2/s
    Translational propulsive velocity: {ag_v_propulse:.1} µm/s

  Derived simulation parameters:
    Agents:
      Rotational diffusion randomisation timescale: {t_rot_diff:.1} s
      System crossing time (s): {t_cross:.1}",
            dt = self.parameters.dt,
            l = 1e6 * self.parameters.boundaries.l(),
            d_trans_diff = 1e12 * self.parameters.agent_translational_diffusion_coefficient,
            d_rot_diff = self.parameters.agent_rotational_diffusion_coefficient,
            ag_v_propulse = 1e6 * self.parameters.agent_propulsion_speed,
            ag_radius = 1e6 * self.parameters.agent_radius,
            t_rot_diff = std::f64::consts::PI * std::f64::consts::PI
                / (4.0 * self.parameters.agent_rotational_diffusion_coefficient),
            t_cross = self.parameters.boundaries.l() / self.parameters.agent_propulsion_speed,
        );
    }
}
