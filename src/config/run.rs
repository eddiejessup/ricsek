use crate::{cuda, dynamics::brownian::brownian_distr};
use rand_distr::Normal;

use super::setup::parameters::simulation::SimParams;

pub struct RunParams {
    pub t_max: f64,
    pub dstep_view: usize,
    pub run_id: usize,
}

pub struct RunContext {
    pub trans_diff_dist: Normal<f64>,
    pub rot_diff_dist: Normal<f64>,
    pub gpu_fluid_context: cuda::fluid::CudaFluidContext,
    pub gpu_electro_context: cuda::electro::CudaElectroContext,
}

impl RunContext {
    pub fn new(sim_params: &SimParams, n_agents: usize) -> Self {
        let trans_diff_dist = brownian_distr(
            sim_params.agent_translational_diffusion_coefficient,
            sim_params.dt,
        );
        let rot_diff_dist = brownian_distr(
            sim_params.agent_rotational_diffusion_coefficient,
            sim_params.dt,
        );

        let gpu_fluid_context = cuda::fluid::CudaFluidContext::new(
            n_agents as u32,
            2 * n_agents as u32, // TODO: Assume two stokeslets per agent for now.
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
            trans_diff_dist,
            rot_diff_dist,
            gpu_fluid_context,
            gpu_electro_context,
        }
    }
}
