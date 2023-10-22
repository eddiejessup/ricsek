#![allow(non_snake_case)]

use core::panic;

use nalgebra::{Point3, Vector3};

use crate::config::setup::parameters::singularities::{Singularity, SingularityParams};

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

pub struct LinearFieldsContext {
    device_data: *mut DeviceData,
    num_eval_points: usize,
}

impl LinearFieldsContext {
    pub fn new(num_eval_points: usize, num_singularities: usize, l: Vector3<f64>) -> Self {
        let device_data = unsafe {
            init(
                num_eval_points as u32,
                num_singularities as u32,
                l.x as f32,
                l.y as f32,
                l.z as f32,
            )
        };
        Self {
            device_data,
            num_eval_points,
        }
    }

    pub fn evaluate(
        &self,
        eval_points: &[(usize, Point3<f64>)],
        singularities: &[(usize, Singularity)],
        threads_per_blockAxis: u32,
    ) -> Vec<Vector3<f64>> {
        let mut eval_points_c: Vec<ObjectPoint> = eval_points
            .iter()
            .map(|(object_id, position)| ObjectPoint {
                object_id: *object_id as u32,
                position: V3 {
                    x: position.x as f32,
                    y: position.y as f32,
                    z: position.z as f32,
                },
            })
            .collect();

        let mut stokeslets_c: Vec<Stokeslet> = singularities
            .iter()
            .map(|(object_id, singularity)| {
                let object_point = ObjectPoint {
                    object_id: *object_id as u32,
                    position: V3 {
                        x: singularity.point.x as f32,
                        y: singularity.point.y as f32,
                        z: singularity.point.z as f32,
                    },
                };
                match singularity.params {
                    SingularityParams::Stokeslet { a } => Stokeslet {
                        object_point,
                        force: V3 {
                            x: a.x as f32,
                            y: a.y as f32,
                            z: a.z as f32,
                        },
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

        let mut eval_point_velocities = vec![
            V3 {
                x: 0.0,
                y: 0.0,
                z: 0.0
            };
            self.num_eval_points
        ];

        unsafe {
            evaluateStokeslets(
                self.device_data,
                eval_points_c.as_mut_ptr(),
                stokeslets_c.as_mut_ptr(),
                eval_point_velocities.as_mut_ptr(),
                threads_per_blockAxis,
            );
        }

        eval_point_velocities
            .iter()
            .map(|v| Vector3::new(v.x as f64, v.y as f64, v.z as f64))
            .collect()
    }
}

impl Drop for LinearFieldsContext {
    fn drop(&mut self) {
        unsafe {
            finalize(self.device_data);
        }
    }
}

// #[cfg(test)]
// mod tests {

//     use super::*;

//     // #[test]
// }
