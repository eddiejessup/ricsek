pub mod agents;
pub mod parameters;

use std::{error::Error, fs::File, io::Read, path::Path};

use log::info;
use nalgebra::Point3;

use crate::geometry::{arange, grid_2d, grid_3d, linspace};

use self::{agents::AgentInitializationConfig, parameters::simulation::SimParams};

#[derive(serde::Serialize, serde::Deserialize)]
pub struct SamplingConfig {
    pub n_sample_points: u32,
}

#[derive(serde::Serialize, serde::Deserialize)]
struct SetupConfigYaml {
    parameters: parameters::ParametersYaml,
    agent_initialization: Vec<AgentInitializationConfig>,
    sampling: SamplingConfig,
}

pub struct SetupConfig {
    pub parameters: SimParams,
    pub agent_initialization: Vec<AgentInitializationConfig>,
    pub sampling: SamplingConfig,
    pub sample_points: Vec<Point3<f64>>,
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

        let mut sample_points = Vec::new();
        sample_points.extend_from_slice(&grid_3d(
            parameters.boundaries.l(),
            config_raw.sampling.n_sample_points as usize,
        ));
        let sample_l = nalgebra::Vector2::new(
            1.2 * parameters.boundaries.l().y,
            1.2 * parameters.boundaries.l().z,
        );
        let step = 2.0;
        for sample_x in vec![parameters.boundaries.l_half().x] {
        //   for sample_x in linspace(
        //     parameters.boundaries.l_half().x - 4.0,
        //     parameters.boundaries.l_half().x,
        //     4,
        // ) {
            for p in grid_2d(sample_l, step) {
                sample_points.push(Point3::new(sample_x, p.x, p.y));
            }
        }

        let config = SetupConfig {
            parameters,
            agent_initialization: config_raw.agent_initialization,
            sampling: config_raw.sampling,
            sample_points,
        };
        Ok(config)
    }

    pub fn print(&self) {
        info!(
            "\
Simulation parameters:
  Environment:
    Timestep: {dt} s
    System length (µm): {l}

  Agents:
    Radius: {ag_radius} µm
    Translational mobility: {ag_mobility:.1} (µm/s)/pN
    Translational diffusion rate: {d_trans_diff:.1} µm^2/s
    Rotational diffusion rate: {d_rot_diff:.1} rad^2/s
    Translational propulsive velocity: {ag_v_propulse:.1} µm/s

  Derived simulation parameters:
    Agents:
      Rotational diffusion randomisation timescale: {t_rot_diff:.1} s
      System crossing time (s): {t_cross:.1}",
            dt = self.parameters.dt,
            l = self.parameters.boundaries.l(),
            d_trans_diff = self.parameters.agent_translational_diffusion_coefficient,
            d_rot_diff = self.parameters.agent_rotational_diffusion_coefficient,
            ag_v_propulse = self.parameters.agent_propulsion_speed(),
            ag_radius = self.parameters.agent_radius,
            ag_mobility = self.parameters.agent_translational_mobility,
            t_rot_diff = std::f64::consts::PI * std::f64::consts::PI
                / (4.0 * self.parameters.agent_rotational_diffusion_coefficient),
            t_cross = self.parameters.boundaries.l() / self.parameters.agent_propulsion_speed(),
        );
    }
}
