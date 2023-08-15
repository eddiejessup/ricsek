use crate::{config::setup::parameters::simulation::BoundaryConfig, state::Agent};
use bevy::prelude::*;
use nalgebra::{Point3, Vector3};

pub fn nalgebra_to_glam_vec(v: &Vector3<f64>) -> Vec3 {
    Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}

#[derive(Resource)]
pub struct Environment {
    pub boundaries: BoundaryConfig,
    pub arrow_length: f64,
    pub length_factor: f64,
}

impl Environment {
    pub fn transform_coord(&self, sd: f64) -> f32 {
        (sd * self.length_factor) as f32
    }

    pub fn invert_coord(&self, sd: f32) -> f64 {
        sd as f64 / self.length_factor
    }

    pub fn transformed_vec3(&self, sd: Point3<f64>) -> Vec3 {
        let st = sd.map(|x| self.transform_coord(x));
        Vec3::new(st.x, st.y, st.z)
    }

    pub fn transformed_l(&self) -> Vec3 {
        self.transformed_vec3(self.boundaries.l().into())
    }

    pub fn inverted_vec3(&self, sd: Vec3) -> Point3<f64> {
        Point3::new(
            self.invert_coord(sd.x),
            self.invert_coord(sd.y),
            self.invert_coord(sd.z),
        )
    }

    pub fn transformed_agent(&self, a: &Agent) -> Transform {
        Transform::from_translation(self.transformed_vec3(a.r))
            .looking_to(nalgebra_to_glam_vec(&a.u), Vec3::Z)
    }
}
