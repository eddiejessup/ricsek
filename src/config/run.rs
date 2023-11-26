use nalgebra::Point3;

use crate::cuda;

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
    pub sample_eval_points: Vec<(u32, nalgebra::Point3<f64>)>,
}

impl RunContext {
    pub fn new(sim_params: &SimParams, n_agents: usize, sample_points: Vec<Point3<f64>>) -> Self {
        let n_singularities = 2 * n_agents as u32; // TODO: Assume two stokeslets per agent for now.

        let gpu_fluid_agent_context = cuda::fluid::CudaFluidContext::new(
            n_agents as u32,
            n_singularities,
            sim_params.boundaries.clone(),
            32,
        );
        let gpu_fluid_sample_context = cuda::fluid::CudaFluidContext::new(
            sample_points.len() as u32,
            n_singularities,
            sim_params.boundaries.clone(),
            32,
        );
        let gpu_electro_context = cuda::electro::CudaElectroContext::new(
            n_agents as u32,
            sim_params.boundaries.clone(),
            sim_params.agent_radius as f32,
            sim_params.agent_object_hertz_force_coefficient,
            5, // Number of approximation points for segment nearest-neighbours.
            32,
        );

        RunContext {
            gpu_fluid_agent_context,
            gpu_fluid_sample_context,
            gpu_electro_context,
            sample_eval_points: sample_points.iter().map(|p| (std::u32::MAX, *p)).collect(),
        }
    }
}
