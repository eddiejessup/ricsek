pub mod agents;
pub mod parameters;

use std::{error::Error, fs::File, io::Read, path::Path};

use nalgebra::{point, Point2};

use crate::math::capsule::Capsule;

use self::parameters::{physical::PhysicalParams, simulation::SimParams, Parameters};

#[derive(serde::Serialize, serde::Deserialize)]
#[serde(tag = "type")]
enum ParametersYaml {
    Physical(PhysicalParams),
    Simulation(SimParams),
}

#[derive(serde::Serialize, serde::Deserialize)]
struct ConfigYaml {
    parameters: ParametersYaml,
    obstacles: Vec<CapsuleYaml>,
    agent_initialization: AgentInitializationConfig,
}

#[derive(serde::Serialize, serde::Deserialize, Debug)]
struct PointYaml {
    x: f64,
    y: f64,
}

#[derive(serde::Serialize, serde::Deserialize, Debug)]
struct CapsuleYaml {
    start: Point2<f64>,
    end: Point2<f64>,
    radius: f64,
}

pub struct SetupConfig {
    pub parameters: Parameters,
    pub obstacles: Vec<crate::math::capsule::Capsule>,
    pub agent_initialization: AgentInitializationConfig,
}

#[derive(serde::Serialize, serde::Deserialize)]
#[serde(tag = "type")]
pub enum AgentInitializationConfig {
    RandomUniformByAreaNumberDensity(AgentAreaNumberDensityConfig),
    RandomUniformByNumber(AgentNumberConfig),
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct AgentAreaNumberDensityConfig {
    pub area_number_density: f64,
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct AgentNumberConfig {
    pub number: usize,
}

impl SetupConfig {
    pub fn parse<P: AsRef<Path>>(path: P) -> Result<Self, Box<dyn Error>> {
        let mut file = File::open(path)?;
        let mut contents = String::new();
        file.read_to_string(&mut contents)?;
        let config_raw: ConfigYaml = serde_yaml::from_str(&contents)?;
        let parameters = match config_raw.parameters {
            ParametersYaml::Physical(physical_params) => Parameters {
                sim_params: physical_params.as_params(),
                physical_params: Some(physical_params),
            },
            ParametersYaml::Simulation(sim_params) => Parameters {
                sim_params,
                physical_params: None,
            },
        };
        let config = SetupConfig {
            parameters,
            obstacles: config_raw
                .obstacles
                .iter()
                .map(
                    |CapsuleYaml {
                         start: s,
                         end: e,
                         radius,
                     }| {
                        Capsule::new(point!(s.x, s.y), point!(e.x, e.y), *radius)
                    },
                )
                .collect(),
            agent_initialization: config_raw.agent_initialization,
        };
        Ok(config)
    }

    pub fn print(&self) {
        match &self.parameters.physical_params {
            Some(physical_params) => {
                println!(
                    "\
Physical parameters:
  Environment:
    Timestep: {dt} s
    Temperature: {temp} K
    System length: {l} µm
    Viscosity: {viscosity} mPa·s

  Agents:
    Propulsive force: {ag_f_propulse} pN
    Dipole strength: {ag_dipole_strength} pN·µm
    Effective radius: {ag_radius} µm

Derived parameters:
  Agents:
    Translational mobility: {ag_trans_mobility:.1} (µm/s)/pN",
                    dt = physical_params.dt,
                    temp = physical_params.fluid_temperature,
                    l = 1e6 * physical_params.l,
                    viscosity = 1e3 * physical_params.fluid_viscosity,
                    ag_f_propulse = 1e12 * physical_params.agent_propulsion_force,
                    ag_dipole_strength = 1e18 * physical_params.agent_propulsion_dipole_strength,
                    ag_radius = 1e6 * physical_params.agent_radius,
                    ag_trans_mobility =
                        1e-6 * physical_params.agent_stokes_translational_mobility(),
                );
            }
            None => {}
        }
        let sim_params = &self.parameters.sim_params;
        println!(
            "\
Environment:
Timestep: {dt} s
System length: {l} µm

Agents:
Effective radius: {ag_radius} µm
Translational diffusion rate: {d_trans_diff:.1} µm^2/s
Rotational diffusion rate: {d_rot_diff:.1} rad^2/s
Translational propulsive velocity: {ag_v_propulse:.1} µm/s

Computed derived parameters (for info only):
Agents:
Rotational diffusion randomisation timescale: {t_rot_diff:.1} s
System crossing time: {t_cross:.1} s",
            dt = sim_params.dt,
            l = 1e6 * sim_params.l,
            d_trans_diff = 1e12 * sim_params.agent_translational_diffusion_coefficient,
            d_rot_diff = sim_params.agent_rotational_diffusion_coefficient,
            ag_v_propulse = 1e6 * sim_params.agent_propulsion_speed,
            ag_radius = 1e6 * sim_params.agent_radius,
            t_rot_diff = std::f64::consts::PI * std::f64::consts::PI
                / (4.0 * sim_params.agent_rotational_diffusion_coefficient),
            t_cross = sim_params.l / sim_params.agent_propulsion_speed,
        );
    }
}
