use nalgebra::Point3;

use crate::{
    cuda::{self},
    geometry::point::ObjectPoint,
};

use super::setup::parameters::simulation::SimParams;

pub struct RunParams {
    pub t_max: f64,
    pub dstep_view: usize,
    pub run_id: usize,
}

pub struct RunContext {
    pub gpu_fluid_agent_context: cuda::fluid::CudaFluidContext,
    pub gpu_fluid_sample_context: cuda::fluid::CudaFluidContext,
    pub gpu_electro_context: cuda::electro::CudaElectroContext,
    pub sample_eval_points: Vec<ObjectPoint>,
}

impl RunContext {
    pub fn new(sim_params: &SimParams, n_agents: usize, sample_points: Vec<Point3<f64>>) -> Self {
        // let n_singularities_per_agent = 6; // TODO: Assume some number of singularities per agent for now.
        let n_singularities_per_agent = 12; // TODO: Assume some number of singularities per agent for now.
        let n_singularities = n_singularities_per_agent * n_agents as u32;

        let gpu_fluid_agent_context = cuda::fluid::CudaFluidContext::new(
            n_agents as u32,
            n_singularities,
            &sim_params.boundaries,
            32,
        );
        let gpu_fluid_sample_context = cuda::fluid::CudaFluidContext::new(
            sample_points.len() as u32,
            n_singularities,
            &sim_params.boundaries,
            32,
        );
        let gpu_electro_context = cuda::electro::CudaElectroContext::new(
            n_agents as u32,
            &sim_params.boundaries,
            sim_params.agent_radius as f32,
            sim_params.agent_object_hertz_force_coefficient,
            5, // Number of approximation points for segment nearest-neighbours.
            32,
        );

        RunContext {
            gpu_fluid_agent_context,
            gpu_fluid_sample_context,
            gpu_electro_context,
            sample_eval_points: sample_points
                .iter()
                .map(|p| ObjectPoint::point_object(std::u32::MAX, *p))
                .collect(),
        }
    }
}
