pub mod contexts {
    use nalgebra::Vector3;

    use crate::{
        config::setup::parameters::{common, singularities},
        dynamics::common::Wrench,
        geometry::{line_segment, point},
    };

    pub trait FluidContextTrait {
        fn new(
            num_eval_points: u32,
            num_singularities: u32,
            boundary_config: &common::BoundaryConfig,
            threads_per_block_axis: u32,
        ) -> Self;

        fn evaluate(
            &self,
            eval_points: &[point::ObjectPoint],
            singularities: &[(point::ObjectPoint, singularities::SingularityParams)],
        ) -> Vec<(Vector3<f64>, Vector3<f64>)>;
    }

    pub trait ElectroContextTrait {
        fn new(
            num_segments: u32,
            boundary_config: &common::BoundaryConfig,
            radius: f32,
            electro_coeff: f64,
            num_approx_points: u32,
            threads_per_block_axis: u32,
        ) -> Self;

        fn evaluate(&self, segments: &[line_segment::LineSegment]) -> Vec<Wrench>;
    }
}

#[cfg(feature = "cuda")]
mod numerics_context {
    // Just used by sibling modules.
    pub mod cuda_interface {
        #![allow(non_snake_case)]
        #![allow(non_upper_case_globals)]

        use nalgebra::{Point3, Vector3};

        use crate::{config::setup::parameters::common, geometry::point};

        include!(concat!(env!("OUT_DIR"), "/cuda_interface_bindings.rs"));

        pub fn zero_v3() -> V3 {
            V3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }
        }

        pub fn zero_v3_pair() -> V3Pair {
            V3Pair {
                a: zero_v3(),
                b: zero_v3(),
            }
        }

        pub fn vec_as_V3(p: Vector3<f64>) -> V3 {
            V3 {
                x: p.x as f32,
                y: p.y as f32,
                z: p.z as f32,
            }
        }

        pub fn point_as_V3(p: Point3<f64>) -> V3 {
            V3 {
                x: p.x as f32,
                y: p.y as f32,
                z: p.z as f32,
            }
        }

        pub fn v3_as_vec(v: V3) -> Vector3<f64> {
            Vector3::new(v.x as f64, v.y as f64, v.z as f64)
        }

        pub fn boundary_config_to_c(bc: &common::BoundaryConfig) -> BoundaryConfig {
            BoundaryConfig {
                lengths: vec_as_V3(bc.l()),
                closed_x: bc.0.x.closed,
                closed_y: bc.0.y.closed,
                closed_z: bc.0.z.closed,
            }
        }

        pub fn object_point_to_c(op: &point::ObjectPoint) -> ObjectPoint {
            ObjectPoint {
                object_id: op.object_id,
                position_com: point_as_V3(op.position_com),
                position: point_as_V3(op.position),
            }
        }
    }

    pub mod electro {
        use crate::geometry::line_segment;
        use crate::{config::setup::parameters::common::BoundaryConfig, dynamics::common::Wrench};

        use super::cuda_interface::{
            boundary_config_to_c, electro_evaluate, electro_finalize, electro_init, point_as_V3,
            v3_as_vec, zero_v3_pair, ElectroDeviceData, V3Pair,
        };

        pub struct ElectroContext {
            device_data: *mut ElectroDeviceData,
            num_segments: u32,
            electro_coeff: f64,
            num_approx_points: u32,
            threads_per_block_axis: u32,
        }

        impl ElectroContextTrait for ElectroContext {
            fn new(
                num_segments: u32,
                boundary_config: &BoundaryConfig,
                radius: f32,
                electro_coeff: f64,
                num_approx_points: u32,
                threads_per_block_axis: u32,
            ) -> Self {
                let device_data = unsafe {
                    electro_init(num_segments, boundary_config_to_c(boundary_config), radius)
                };
                Self {
                    device_data,
                    num_segments,
                    electro_coeff,
                    num_approx_points,
                    threads_per_block_axis,
                }
            }

            fn evaluate(&self, segments: &[line_segment::LineSegment]) -> Vec<Wrench> {
                if segments.len() as u32 != self.num_segments {
                    panic!(
                        "segments.len() = {} but self.num_segments = {}",
                        segments.len(),
                        self.num_segments
                    );
                }

                let mut segments_c: Vec<V3Pair> = segments
                    .iter()
                    .map(|s| V3Pair {
                        a: point_as_V3(s.start),
                        b: point_as_V3(s.end),
                    })
                    .collect();

                let mut net_wrenches_c = vec![zero_v3_pair(); self.num_segments as usize];

                unsafe {
                    electro_evaluate(
                        self.device_data,
                        segments_c.as_mut_ptr(),
                        net_wrenches_c.as_mut_ptr(),
                        self.threads_per_block_axis,
                        self.num_approx_points,
                    );
                }

                net_wrenches_c
                    .iter()
                    .map(|w| Wrench {
                        force: v3_as_vec(w.a).scale(self.electro_coeff),
                        torque: v3_as_vec(w.b).scale(self.electro_coeff),
                    })
                    .collect()
            }
        }

        impl Drop for ElectroContext {
            fn drop(&mut self) {
                unsafe {
                    electro_finalize(self.device_data);
                }
            }
        }

        #[cfg(test)]
        mod tests {
            use crate::{
                config::setup::parameters::common::{AxisBoundaryConfig, BoundaryConfig},
                dynamics::agent::capsules_capsules_electro,
                geometry::line_segment::LineSegment,
            };

            use super::ElectroContext;
            use approx::assert_relative_eq;
            use nalgebra::{Point3, Vector3};

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
                let radius = 0.5;
                let num_approx_points = 5;
                let electro_coeff = 1.0;

                // Define two positions
                let segments = vec![
                    LineSegment {
                        start: Point3::new(-0.5, 0.0, 0.0),
                        end: Point3::new(-0.5, 1.0, 0.0),
                    },
                    LineSegment {
                        start: Point3::new(0.4, 0.0, 0.0),
                        end: Point3::new(0.4, 1.0, 0.0),
                    },
                ];

                let gpu_context = ElectroContext::new(
                    segments.len() as u32,
                    &boundary_config,
                    radius,
                    electro_coeff,
                    num_approx_points,
                    32,
                );

                let vs_gpu = gpu_context.evaluate(&segments);
                let vs_cpu = capsules_capsules_electro(&segments, radius as f64, electro_coeff);

                assert!(
                    vs_gpu.len() == vs_cpu.len(),
                    "vs_gpu={:?}, vs_cpu={:?}",
                    vs_gpu,
                    vs_cpu
                );

                for (w_gpu, w_cpu) in vs_gpu.iter().zip(vs_cpu.iter()) {
                    assert_relative_eq!(w_gpu.force, w_cpu.force, epsilon = 1e-6);
                    assert_relative_eq!(w_gpu.torque, w_cpu.torque, epsilon = 1e-6);
                }
            }
        }
    }

    pub mod fluid {
        use core::panic;

        use nalgebra::Vector3;

        use crate::{
            config::setup::parameters::{common::BoundaryConfig, singularities},
            geometry::point,
        };

        use super::cuda_interface::{
            self, boundary_config_to_c, fluid_evaluate, fluid_finalize, fluid_init,
            object_point_to_c, v3_as_vec, vec_as_V3, zero_v3_pair, FluidDeviceData, ObjectPoint,
            Singularity, SingularityParams, V3Pair,
        };

        pub struct FluidContext {
            device_data: *mut FluidDeviceData,
            num_eval_points: u32,
            num_singularities: u32,
            threads_per_block_axis: u32,
        }

        impl FluidContextTrait for FluidContext {
            fn new(
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

            fn evaluate(
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

                let mut singularities_c: Vec<Singularity> = singularities
                    .iter()
                    .map(|(object_pt, params)| {
                        let object_point = object_point_to_c(object_pt);
                        match params {
                            singularities::SingularityParams::Stokeslet { a } => {
                                cuda_interface::Singularity {
                                    object_point,
                                    singularity_type: cuda_interface::SingularityType_STOKESLET,
                                    params: SingularityParams {
                                        strength: vec_as_V3(*a),
                                    },
                                }
                            }
                            singularities::SingularityParams::StokesDoublet { a, b } => {
                                cuda_interface::Singularity {
                                    object_point,
                                    singularity_type:
                                        cuda_interface::SingularityType_STOKES_DOUBLET,
                                    params: SingularityParams {
                                        components: V3Pair {
                                            a: vec_as_V3(*a),
                                            b: vec_as_V3(*b),
                                        },
                                    },
                                }
                            }
                            singularities::SingularityParams::Rotlet { c } => {
                                cuda_interface::Singularity {
                                    object_point,
                                    singularity_type: cuda_interface::SingularityType_ROTLET,
                                    params: SingularityParams {
                                        strength: vec_as_V3(*c),
                                    },
                                }
                            }
                            singularities::SingularityParams::Stresslet { a, b } => {
                                cuda_interface::Singularity {
                                    object_point,
                                    singularity_type: cuda_interface::SingularityType_STRESSLET,
                                    params: SingularityParams {
                                        components: V3Pair {
                                            a: vec_as_V3(*a),
                                            b: vec_as_V3(*b),
                                        },
                                    },
                                }
                            }
                            singularities::SingularityParams::PotentialDoublet { d } => {
                                cuda_interface::Singularity {
                                    object_point,
                                    singularity_type:
                                        cuda_interface::SingularityType_POTENTIAL_DOUBLET,
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
                        singularities_c.as_mut_ptr(),
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

        impl Drop for FluidContext {
            fn drop(&mut self) {
                unsafe {
                    fluid_finalize(self.device_data);
                }
            }
        }

        #[cfg(test)]
        mod tests {
            use super::FluidContext;
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

                let gpu_context = FluidContext::new(
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
    }
}
#[cfg(not(feature = "cuda"))]
mod numerics_context {
    use log::warn;
    use nalgebra::Vector3;
    use num_traits::Zero;

    use crate::{
        config::setup::parameters::{common, singularities},
        dynamics::{boundary::minimum_image_vector, common::Wrench},
        geometry::{line_segment, point},
    };

    fn evaluate_stokeslet_v(
        strength: &Vector3<f64>,
        r: &Vector3<f64>,
        r_length: f64,
    ) -> Vector3<f64> {
        // (a / |r|) + (a.dot(r) * r / |r|^3)
        let v_f = strength.unscale(r_length);
        let v_r = r.scale(strength.dot(r) / (r_length * r_length * r_length));
        v_f + v_r
    }

    use super::contexts::{ElectroContextTrait, FluidContextTrait};
    pub struct FluidContext {
        boundaries: common::BoundaryConfig,
        num_eval_points: u32,
    }

    impl FluidContextTrait for FluidContext {
        fn new(
            num_eval_points: u32,
            _num_singularities: u32,
            boundaries: &common::BoundaryConfig,
            _threads_per_block_axis: u32,
        ) -> Self {
            FluidContext {
                boundaries: boundaries.clone(),
                num_eval_points,
            }
        }

        fn evaluate(
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
            // Overall plan:
            // For each eval point, compute the linear and angular velocity pair ('twist') due to each singularity.
            // Sum the twists to get the total twist.
            // For a first implementation, just iterate naively over all combinations, except when they belong to the same object.
            // And because we are moving towards a pure-stokeslet system, only support stokeslet singularities.
            eval_points
                .iter()
                .map(|p| {
                    let mut v = Vector3::zeros();
                    for (sp, s) in singularities.iter() {
                        if sp.object_id == p.object_id {
                            continue;
                        }
                        match s {
                            singularities::SingularityParams::Stokeslet { a } => {
                                // Vector from singularity to eval point.
                                let r = minimum_image_vector(
                                    p.position,
                                    p.position_com,
                                    sp.position,
                                    sp.position_com,
                                    &self.boundaries,
                                );

                                let r_length = r.magnitude();
                                if r_length.is_nan() || r_length.is_infinite() || r_length < 1e-6 {
                                    warn!("Warning: NaN or infinite r_length detected: {:?}", r);
                                };
                                v += evaluate_stokeslet_v(a, &r, r_length);
                            }
                            _ => panic!("Unsupported singularity type"),
                        }
                    }
                    (v, Vector3::zeros())
                })
                .collect()
        }
    }

    pub struct ElectroContext;

    impl ElectroContextTrait for ElectroContext {
        fn new(
            _num_segments: u32,
            _boundary_config: &common::BoundaryConfig,
            _radius: f32,
            _electro_coeff: f64,
            _num_approx_points: u32,
            _threads_per_block_axis: u32,
        ) -> Self {
            ElectroContext
        }

        // Stub: return zero-wrenches of same size as input.
        fn evaluate(&self, _segments: &[line_segment::LineSegment]) -> Vec<Wrench> {
            vec![Wrench::zero(); _segments.len()]
        }
    }
}

pub use contexts::*;
pub use numerics_context::*;
