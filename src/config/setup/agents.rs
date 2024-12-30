use std::f64::consts::PI;

use crate::geometry::point::{random_unit_vector, random_vector};
use crate::state::{self, *};

use nalgebra::{Point3, UnitVector3, Vector3};
use rand::distributions::Uniform;
use rand_distr::Distribution;

#[derive(serde::Serialize, serde::Deserialize, Clone)]
#[serde(tag = "type")]
pub enum AgentInitializationConfig {
    RandomUniformByVolumeNumberDensity(AgentVolumeNumberDensityConfig),
    RandomUniformByNumber(AgentNumberConfig),
    Explicit(ExplicitAgentsConfig),
}

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct AgentVolumeNumberDensityConfig {
    pub volume_number_density: f64,
}

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct AgentNumberConfig {
    pub number: usize,
}

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct ExplicitAgentsConfig {
    pub agents: Vec<ExplicitAgentConfig>,
}

#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct ExplicitAgentConfig {
    r: Point3<f64>,
    u: UnitVector3<f64>,
}

pub fn random_uniform_orientations_iter<R: rand::Rng>(
    rng: &mut R,
) -> impl Iterator<Item = UnitVector3<f64>> + '_ {
    std::iter::repeat_with(|| random_unit_vector(rng))
}

pub fn random_uniform_position_iter<R: rand::Rng>(
    rng: &mut R,
    l: Vector3<f64>,
) -> impl Iterator<Item = Point3<f64>> + '_ {
    let r_distr = Uniform::new(-0.5, 0.5);
    std::iter::repeat_with(move || random_vector(rng, r_distr).component_mul(&l).into())
}

pub fn random_uniform_angle_iter<R: rand::Rng>(rng: &mut R) -> impl Iterator<Item = f64> + '_ {
    let th_distr = Uniform::new(-PI, PI);
    th_distr.sample_iter(rng)
}

fn random_uniform_agents<R: rand::Rng>(
    rng: &mut R,
    n: usize,
    l: Vector3<f64>,
    d: f64,
) -> Vec<state::Agent> {
    let rs: Vec<Point3<f64>> = random_uniform_position_iter(rng, l.add_scalar(-2.0 * d))
        .take(n)
        .collect();
    let us: Vec<UnitVector3<f64>> = random_uniform_orientations_iter(rng).take(n).collect();
    let tail_phases: Vec<f64> = random_uniform_angle_iter(rng).take(n).collect();
    (0..n)
        .map(|i| Agent::new(rs[i], us[i].scale(d), tail_phases[i]))
        .collect()
}

pub fn initialize_agents<R: rand::Rng>(
    rng: &mut R,
    config: AgentInitializationConfig,
    l: Vector3<f64>,
    d: f64,
) -> Vec<Agent> {
    match config {
        AgentInitializationConfig::RandomUniformByVolumeNumberDensity(
            AgentVolumeNumberDensityConfig {
                volume_number_density,
            },
        ) => {
            let n = (volume_number_density * l.x * l.y * l.z).round() as usize;
            random_uniform_agents(rng, n, l, d)
        }
        AgentInitializationConfig::RandomUniformByNumber(AgentNumberConfig { number }) => {
            random_uniform_agents(rng, number, l, d)
        }
        AgentInitializationConfig::Explicit(ExplicitAgentsConfig { agents }) => agents
            .iter()
            .map(|ExplicitAgentConfig { r, u }| {
                Agent::new(
                    *r,
                    u.scale(d),
                    random_uniform_angle_iter(rng).next().unwrap(),
                )
            })
            .collect(),
    }
}
