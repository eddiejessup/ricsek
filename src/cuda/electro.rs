use crate::geometry::line_segment;
use crate::{config::setup::parameters::common::BoundaryConfig, dynamics::common::Wrench};

use super::interface::{
    boundary_config_to_c, electro_evaluate, electro_finalize, electro_init, point_as_V3, v3_as_vec,
    zero_v3_pair, ElectroDeviceData, V3Pair,
};

pub struct CudaElectroContext {
    device_data: *mut ElectroDeviceData,
    num_segments: u32,
    electro_coeff: f64,
    num_approx_points: u32,
    threads_per_block_axis: u32,
}

impl CudaElectroContext {
    pub fn new(
        num_segments: u32,
        boundary_config: BoundaryConfig,
        radius: f32,
        electro_coeff: f64,
        num_approx_points: u32,
        threads_per_block_axis: u32,
    ) -> Self {
        let device_data =
            unsafe { electro_init(num_segments, boundary_config_to_c(boundary_config), radius) };
        Self {
            device_data,
            num_segments,
            electro_coeff,
            num_approx_points,
            threads_per_block_axis,
        }
    }

    pub fn evaluate(&self, segments: &[line_segment::LineSegment]) -> Vec<Wrench> {
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

impl Drop for CudaElectroContext {
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
        dynamics::agent::capsules_capsules_electro, geometry::line_segment::LineSegment,
    };

    use super::CudaElectroContext;
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

        let gpu_context = CudaElectroContext::new(
            segments.len() as u32,
            boundary_config,
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
