use core::panic;

use nalgebra::Vector3;

use crate::{
    config::setup::parameters::{common::BoundaryConfig, singularities::SingularityParams},
    geometry::point,
};

use super::interface::{
    boundary_config_to_c, fluid_evaluate, fluid_finalize, fluid_init, object_point_to_c, v3_as_vec,
    vec_as_V3, zero_v3_pair, FluidDeviceData, ObjectPoint, Stokeslet, V3Pair,
};

pub struct CudaFluidContext {
    device_data: *mut FluidDeviceData,
    num_eval_points: u32,
    num_singularities: u32,
    threads_per_block_axis: u32,
}

impl CudaFluidContext {
    pub fn new(
        num_eval_points: u32,
        num_singularities: u32,
        boundary_config: &BoundaryConfig,
        threads_per_block_axis: u32,
    ) -> Self {
        let device_data = unsafe {
            fluid_init(
                num_eval_points,
                num_singularities,
                boundary_config_to_c(boundary_config),
            )
        };
        Self {
            device_data,
            num_eval_points,
            num_singularities,
            threads_per_block_axis,
        }
    }

    pub fn evaluate(
        &self,
        eval_points: &[point::ObjectPoint],
        singularities: &[(point::ObjectPoint, SingularityParams)],
    ) -> Vec<(Vector3<f64>, Vector3<f64>)> {
        if eval_points.len() as u32 != self.num_eval_points {
            panic!(
                "eval_points.len() = {} but self.num_eval_points = {}",
                eval_points.len(),
                self.num_eval_points
            );
        }
        if singularities.len() as u32 != self.num_singularities {
            panic!(
                "singularities.len() = {} but self.num_singularities = {}",
                singularities.len(),
                self.num_singularities
            );
        }

        let mut eval_points_c: Vec<ObjectPoint> =
            eval_points.iter().map(|op| object_point_to_c(op)).collect();

        let mut stokeslets_c: Vec<Stokeslet> = singularities
            .iter()
            .map(|(object_pt, params)| {
                let object_point = object_point_to_c(object_pt);
                match params {
                    SingularityParams::Stokeslet { a } => Stokeslet {
                        object_point,
                        force: vec_as_V3(*a),
                    },
                    SingularityParams::StokesDoublet { a: _, b: _ } => {
                        panic!("StokesDoublet not supported")
                    }
                    SingularityParams::Rotlet { c: _ } => panic!("Rotlet not supported"),
                    SingularityParams::Stresslet { a: _, b: _ } => {
                        panic!("Stresslet not supported")
                    }
                    SingularityParams::PotentialDoublet { d: _ } => {
                        panic!("PotentialDoublet not supported")
                    }
                }
            })
            .collect();

        let mut eval_point_twists_c: Vec<V3Pair> =
            vec![zero_v3_pair(); self.num_eval_points as usize];

        unsafe {
            fluid_evaluate(
                self.device_data,
                eval_points_c.as_mut_ptr(),
                stokeslets_c.as_mut_ptr(),
                eval_point_twists_c.as_mut_ptr(),
                self.threads_per_block_axis,
            );
        }

        eval_point_twists_c
            .iter()
            .map(|twist| (v3_as_vec(twist.a), v3_as_vec(twist.b)))
            .collect()
    }
}

impl Drop for CudaFluidContext {
    fn drop(&mut self) {
        unsafe {
            fluid_finalize(self.device_data);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::CudaFluidContext;
    use approx::assert_relative_eq;
    use nalgebra::{Point3, Vector3};

    use crate::{
        config::setup::parameters::{
            common::{AxisBoundaryConfig, BoundaryConfig},
            singularities::{singularities_fluid_v_multi, SingularityParams},
        },
        geometry::point,
    };

    #[test]
    fn test_happy() {
        // Define box dimensions
        let boundary_config = BoundaryConfig(Vector3::new(
            AxisBoundaryConfig {
                l: 100.0,
                closed: true,
            },
            AxisBoundaryConfig {
                l: 100.0,
                closed: true,
            },
            AxisBoundaryConfig {
                l: 100.0,
                closed: true,
            },
        ));
        // Define two positions
        let eval_points = vec![
            point::ObjectPoint::point_object(0, Point3::new(0.0, 0.0, 0.0)),
            point::ObjectPoint::point_object(1, Point3::new(1.0, 0.0, 0.0)),
        ];

        let singularities = vec![
            (
                eval_points[0].clone(),
                SingularityParams::Stokeslet {
                    a: Vector3::new(1.0, 0.0, 0.0),
                },
            ),
            (
                eval_points[1].clone(),
                SingularityParams::Stokeslet {
                    a: Vector3::new(-1.0, 0.0, 0.0),
                },
            ),
        ];

        let gpu_context = CudaFluidContext::new(
            eval_points.len() as u32,
            singularities.len() as u32,
            &boundary_config,
            32,
        );

        let vs_gpu = gpu_context.evaluate(&eval_points, &singularities);
        let vs_cpu = singularities_fluid_v_multi(&eval_points, &singularities);

        assert!(
            vs_gpu.len() == vs_cpu.len(),
            "vs_gpu={:?}, vs_cpu={:?}",
            vs_gpu,
            vs_cpu
        );

        for (t_gpu, t_cpu) in vs_gpu.iter().zip(vs_cpu.iter()) {
            assert_relative_eq!(t_gpu.0, t_cpu.0, epsilon = 1e-6);
            assert_relative_eq!(t_gpu.1, t_cpu.1, epsilon = 1e-6);
        }
    }
}
