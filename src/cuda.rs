#![allow(non_snake_case)]

use nalgebra::{Point3, Vector3};

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

pub struct LinearFieldsContext {
    device_data: *mut DeviceData,
    num_elements: usize,
}

impl LinearFieldsContext {
    pub fn new(num_elements: usize, l: Vector3<f64>) -> Self {
        let device_data = unsafe { init(num_elements as u64, l.x, l.y, l.z) };
        Self {
            device_data,
            num_elements,
        }
    }

    pub fn populate_pairwise_distances(&self, positions: &mut [Point3<f64>], threads_per_block: u32) {
        let mut flat_positions = vector3_vec_to_c_array(positions);
        println!("flat_positions = {:?}", flat_positions);
        unsafe {
          populatePairwiseDistances(
                self.device_data,
                flat_positions.as_mut_ptr(),
                threads_per_block,
            );
        }
    }

    pub fn fetch_pairwise_distances(&self) -> Vec<Vector3<f64>> {
        let mut flat_distances = vec![0.0; self.num_elements * self.num_elements * 3];
      unsafe {
        fetchPairwiseDistances(
              self.device_data,
              flat_distances.as_mut_ptr(),
          );
        }
        println!("flat_distances = {:?}", flat_distances);
        c_array_to_vector3_vec(&flat_distances)

  }

    pub fn net_forces(&self, threads_per_block: u32) -> Vec<Vector3<f64>> {
        // Allocate forces array of size `num_elements`.
        let mut flat_forces = vec![0.0; self.num_elements * 3];
        unsafe {
            netForces(
                self.device_data,
                flat_forces.as_mut_ptr(),
                threads_per_block,
            );
        }
        c_array_to_vector3_vec(&flat_forces)
    }
}

fn vector3_vec_to_c_array(xs: &[Point3<f64>]) -> Vec<f64> {
    let mut c_array = Vec::with_capacity(3 * xs.len());
    for x in xs {
        c_array.push(x.coords.x);
        c_array.push(x.coords.y);
        c_array.push(x.coords.z);
    }
    c_array
}

fn c_array_to_vector3_vec(xs: &[f64]) -> Vec<Vector3<f64>> {
    let mut vector3_vec = Vec::with_capacity(xs.len() / 3);
    for i in 0..xs.len() / 3 {
        vector3_vec.push(Vector3::new(xs[3 * i], xs[3 * i + 1], xs[3 * i + 2]));
    }
    vector3_vec
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
