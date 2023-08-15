use crate::math::point::{random_unit_vector, random_vector};
use crate::state::{self, *};
use nalgebra::{Point3, UnitVector3, Vector3};
use rand::distributions::Uniform;
use rand::rngs::ThreadRng;

use super::{
    AgentInitializationConfig, AgentInitializationConfig::*, AgentNumberConfig,
    AgentVolumeNumberDensityConfig, ExplicitAgentConfig,
};

pub fn random_uniform_orientations(rng: &mut ThreadRng, n: usize) -> Vec<UnitVector3<f64>> {
    (0..n).map(|_i| random_unit_vector(rng)).collect()
}

pub fn random_uniform_positions(
    rng: &mut ThreadRng,
    n: usize,
    l: Vector3<f64>,
) -> Vec<Point3<f64>> {
    let r_distr = Uniform::new(-0.5, 0.5);
    (0..n)
        .map(|_i| random_vector(rng, r_distr).component_mul(&l).into())
        .collect()
}

fn random_uniform_agents(rng: &mut ThreadRng, n: usize, l: Vector3<f64>) -> Vec<state::Agent> {
    let rs = random_uniform_positions(rng, n, l);
    let us = random_uniform_orientations(rng, n);
    (0..n).map(|i| Agent { r: rs[i], u: us[i] }).collect()
}

pub fn initialize_agents(
    rng: &mut ThreadRng,
    config: AgentInitializationConfig,
    l: Vector3<f64>,
) -> Vec<Agent> {
    match config {
        RandomUniformByVolumeNumberDensity(AgentVolumeNumberDensityConfig {
            volume_number_density,
        }) => {
            let n = (volume_number_density * l.x * l.y * l.z).round() as usize;
            random_uniform_agents(rng, n, l)
        }
        RandomUniformByNumber(AgentNumberConfig { number }) => {
            random_uniform_agents(rng, number, l)
        }
        Explicit(ExplicitAgentConfig { agents }) => agents,
    }
}
