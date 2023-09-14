use std::f64::consts::PI;

use super::{common::BoundaryConfig, simulation::SimParams};

// Derive JSON deserialize for PhysicalParams.
#[derive(serde::Serialize, serde::Deserialize, Debug)]
pub struct PhysicalParams {
    // Time step.
    pub dt: f64,
    // System.
    pub boundaries: BoundaryConfig,
    pub singularities: Vec<super::singularities::Singularity>,
    // Agent shape.
    pub agent_radius: f64,
    // Length includes the 'caps'.
    pub agent_length: f64,
    pub agent_linear_spring_stiffness: f64,
    // Stiffness of both agents and surfaces.
    // (Used to compute the strength of agent-{agent, surface} electrostatics)
    pub object_stiffness: f64,
    // Fluid.
    pub fluid_temperature: f64,
    pub fluid_viscosity: f64,
    // Agent propulsion.
    // Force the agent applies to the fluid through its propulsion.
    // (Used to compute the agent's empty-fluid velocity.)
    pub agent_propulsion_force: f64,
}

impl PhysicalParams {
  pub fn agent_inter_sphere_length(&self) -> f64 {
    // Get length of the cylinder part.
    let r = self.agent_length - 2.0 * self.agent_radius;
    if r < 0.0 {
        panic!("Agent length must be greater than 2 * agent radius.")
    } else {
        r
    }
}

    fn agent_volume(&self) -> f64 {
      // Volume of a cylinder plus two end-half-spheres.
      (PI * self.agent_radius.powi(2) * self.agent_inter_sphere_length())
      +
      (4.0 / 3.0) * PI * self.agent_radius.powi(3)
    }

    fn agent_effective_radius(&self) -> f64 {
        // Volume of a sphere with the same volume as the agent.
        (3.0 * self.agent_volume() / (4.0 * PI)).powf(1.0 / 3.0)
    }

    // TODO: This assumes a sphere, but we're using a spherocylinder.
    fn agent_stokes_translational_resistance(&self) -> f64 {
        // v = F / 6πηa
        6.0 * PI * self.fluid_viscosity * self.agent_effective_radius()
    }

    fn agent_stokes_rotational_resistance(&self) -> f64 {
        // T = 8πηa^3ω
        8.0 * PI * self.fluid_viscosity * self.agent_effective_radius().powi(3)
    }

    pub fn agent_stokes_translational_mobility(&self) -> f64 {
        1.0 / self.agent_stokes_translational_resistance()
    }

    pub fn agent_stokes_rotational_mobility(&self) -> f64 {
        1.0 / self.agent_stokes_rotational_resistance()
    }

    fn thermal_energy(&self) -> f64 {
        physical_constants::BOLTZMANN_CONSTANT * self.fluid_temperature
    }

    fn agent_stokes_translational_diffusion_coefficient(&self) -> f64 {
        // D = kBT / 6πηa
        self.thermal_energy() / self.agent_stokes_translational_resistance()
    }

    fn agent_stokes_rotational_diffusion_coefficient(&self) -> f64 {
        // Drot = kBT / 8πηa^3
        self.thermal_energy() / self.agent_stokes_rotational_resistance()
    }

    pub fn agent_object_hertz_force_coefficient(&self) -> f64 {
        (4.0 / 3.0) * self.object_stiffness * self.agent_effective_radius().sqrt()
    }

    pub fn as_params(&self) -> SimParams {
        SimParams {
            dt: self.dt,
            boundaries: self.boundaries.clone(),
            singularities: self.singularities.clone(),
            agent_radius: self.agent_radius,
            agent_inter_sphere_length: self.agent_inter_sphere_length(),
            agent_linear_spring_stiffness: self.agent_linear_spring_stiffness,
            agent_translational_mobility: self.agent_stokes_translational_mobility(),
            agent_rotational_mobility: self.agent_stokes_rotational_mobility(),
            agent_propulsion_force: self.agent_propulsion_force,
            agent_translational_diffusion_coefficient: self
                .agent_stokes_translational_diffusion_coefficient(),
            agent_rotational_diffusion_coefficient: self
                .agent_stokes_rotational_diffusion_coefficient(),
            agent_object_hertz_force_coefficient: self.agent_object_hertz_force_coefficient(),
        }
    }
}
