use core::panic;

use nalgebra::Vector3;

use crate::{
    config::setup::parameters::{common::BoundaryConfig, singularities},
    geometry::point,
};

use super::interface::{
    self, boundary_config_to_c, fluid_evaluate, fluid_finalize, fluid_init, object_point_to_c,
    v3_as_vec, vec_as_V3, zero_v3_pair, FluidDeviceData, ObjectPoint, Singularity,
    SingularityParams, V3Pair,
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
        singularities: &[(point::ObjectPoint, singularities::SingularityParams)],
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

        let mut stokeslets_c: Vec<Singularity> = singularities
            .iter()
            .map(|(object_pt, params)| {
                let object_point = object_point_to_c(object_pt);
                match params {
                    singularities::SingularityParams::Stokeslet { a } => interface::Singularity {
                        object_point,
                        singularity_type: interface::SingularityType_STOKESLET,
                        params: SingularityParams {
                            strength: vec_as_V3(*a),
                        },
                    },
                    singularities::SingularityParams::StokesDoublet { a, b } => {
                        interface::Singularity {
                            object_point,
                            singularity_type: interface::SingularityType_STOKES_DOUBLET,
                            params: SingularityParams {
                                components: V3Pair {
                                    a: vec_as_V3(*a),
                                    b: vec_as_V3(*b),
                                },
                            },
                        }
                    }
                    singularities::SingularityParams::Rotlet { c } => interface::Singularity {
                        object_point,
                        singularity_type: interface::SingularityType_ROTLET,
                        params: SingularityParams {
                            strength: vec_as_V3(*c),
                        },
                    },
                    singularities::SingularityParams::Stresslet { a, b } => {
                        interface::Singularity {
                            object_point,
                            singularity_type: interface::SingularityType_STRESSLET,
                            params: SingularityParams {
                                components: V3Pair {
                                    a: vec_as_V3(*a),
                                    b: vec_as_V3(*b),
                                },
                            },
                        }
                    }
                    singularities::SingularityParams::PotentialDoublet { d } => {
                        interface::Singularity {
                            object_point,
                            singularity_type: interface::SingularityType_POTENTIAL_DOUBLET,
                            params: SingularityParams {
                                strength: vec_as_V3(*d),
                            },
                        }
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
            .map(|twist| {
                // Check if any component is NaN or non-finite, and panic if so.
                if twist.a.x.is_nan()
                    || twist.a.y.is_nan()
                    || twist.a.z.is_nan()
                    || twist.b.x.is_nan()
                    || twist.b.y.is_nan()
                    || twist.b.z.is_nan()
                    || twist.a.x.is_infinite()
                    || twist.a.y.is_infinite()
                    || twist.a.z.is_infinite()
                    || twist.b.x.is_infinite()
                    || twist.b.y.is_infinite()
                    || twist.b.z.is_infinite()
                {
                    panic!(
                        "twist.a = {:?}, twist.b = {:?}",
                        v3_as_vec(twist.a),
                        v3_as_vec(twist.b)
                    );
                }

                (v3_as_vec(twist.a), v3_as_vec(twist.b))
            })
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
            singularities::{self, singularities_fluid_v_multi},
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
                singularities::SingularityParams::Stokeslet {
                    a: Vector3::new(1.0, 0.0, 0.0),
                },
            ),
            (
                eval_points[1].clone(),
                singularities::SingularityParams::Stokeslet {
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
