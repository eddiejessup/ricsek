use std::f64::consts::PI;

use nalgebra::{Point3, Vector3};

use crate::geometry::{capsule::Capsule, helix::Helix, line_segment::LineSegment};

use super::{common::BoundaryConfig, simulation::SimParams};

#[derive(serde::Serialize, serde::Deserialize, Debug)]
pub struct PhysicalParams {
    // Time step.
    pub dt: f64,
    // System.
    pub boundaries: BoundaryConfig,
    pub singularities: Vec<super::singularities::Singularity>,
    // Agent shape.
    pub agent_radius: f64,
    //   Length includes the 'caps'.
    pub agent_length: f64,
    // Electrostatics.
    //   Stiffness of both agents and surfaces, used to compute the strength of
    //   agent-{agent, surface} repulsion.
    pub enable_agent_agent_electro: bool,
    pub enable_agent_boundary_electro: bool,
    pub object_stiffness: f64,
    // Fluid.
    pub enable_fluid: bool,
    pub fluid_temperature: f64,
    pub fluid_viscosity: f64,
    // Agent propulsion.
    // Force the agent applies to the fluid through its propulsion.
    // (Used to compute the agent's empty-fluid velocity.)
    pub agent_propulsion_force: f64,
    pub agent_propulsion_torque: f64,
    pub enable_agent_propulsion: bool,
    pub agent_tail: Option<Helix>,
    pub agent_tail_rotation_rate: f64,
    pub agent_tail_n_points: usize,
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
        Capsule {
            segment: LineSegment {
                start: Point3::origin(),
                end: Point3::origin() + Vector3::new(0.0, 0.0, self.agent_inter_sphere_length()),
            },
            radius: self.agent_radius,
        }
        .volume()
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

    pub fn agent_propulsive_stokeslet_strength(&self) -> f64 {
        self.agent_propulsion_force / (8.0 * PI * self.fluid_viscosity)
    }

    pub fn agent_propulsive_rotlet_strength(&self) -> f64 {
        self.agent_propulsion_torque / (8.0 * PI * self.fluid_viscosity)
    }

    fn thermal_energy(&self) -> f64 {
        // Boltzmann in length-units of microns.
        1e12 * physical_constants::BOLTZMANN_CONSTANT * self.fluid_temperature
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
            agent_translational_mobility: self.agent_stokes_translational_mobility(),
            agent_rotational_mobility: self.agent_stokes_rotational_mobility(),
            agent_propulsion_force: self.agent_propulsion_force,
            agent_propulsive_stokeslet_strength: self.agent_propulsive_stokeslet_strength(),
            agent_propulsive_rotlet_strength: self.agent_propulsive_rotlet_strength(),
            agent_translational_diffusion_coefficient: self
                .agent_stokes_translational_diffusion_coefficient(),
            agent_rotational_diffusion_coefficient: self
                .agent_stokes_rotational_diffusion_coefficient(),
            agent_object_hertz_force_coefficient: self.agent_object_hertz_force_coefficient(),
            enable_fluid: self.enable_fluid,
            enable_agent_propulsion: self.enable_agent_propulsion,
            enable_agent_agent_electro: self.enable_agent_agent_electro,
            enable_agent_boundary_electro: self.enable_agent_boundary_electro,
            agent_tail: self.agent_tail.clone(),
            agent_tail_rotation_rate: self.agent_tail_rotation_rate,
            agent_tail_n_points: self.agent_tail_n_points,
        }
    }
}
