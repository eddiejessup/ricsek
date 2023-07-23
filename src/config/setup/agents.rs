use crate::math::point::{random_point, random_unit_vector};
use crate::state::{self, *};
use nalgebra::{Point2, UnitVector2};
use rand::distributions::Uniform;
use rand::rngs::ThreadRng;

use super::{
    AgentAreaNumberDensityConfig, AgentInitializationConfig, AgentInitializationConfig::*,
    AgentNumberConfig,
};

pub fn random_uniform_orientations(rng: &mut ThreadRng, n: usize) -> Vec<UnitVector2<f64>> {
    let th_distr = Uniform::new(-std::f64::consts::PI, std::f64::consts::PI);
    (0..n).map(|_i| random_unit_vector(rng, th_distr)).collect()
}

pub fn random_uniform_positions(rng: &mut ThreadRng, n: usize, l: f64) -> Vec<Point2<f64>> {
    let r_distr = Uniform::new(-l * 0.5, l * 0.5);
    (0..n).map(|_i| random_point(rng, r_distr).into()).collect()
}

fn random_uniform_agents(rng: &mut ThreadRng, n: usize, l: f64) -> Vec<state::Agent> {
    let rs = random_uniform_positions(rng, n, l);
    let us = random_uniform_orientations(rng, n);
    (0..n).map(|i| Agent { r: rs[i], u: us[i] }).collect()
}

pub fn initialize_agents(
    rng: &mut ThreadRng,
    config: &AgentInitializationConfig,
    l: f64,
) -> Vec<Agent> {
    match config {
        RandomUniformByAreaNumberDensity(AgentAreaNumberDensityConfig {
            area_number_density,
        }) => {
            let n = (area_number_density * l.powi(2)).round() as usize;
            random_uniform_agents(rng, n, l)
        }
        RandomUniformByNumber(AgentNumberConfig { number }) => {
            random_uniform_agents(rng, *number, l)
        }
    }
}
