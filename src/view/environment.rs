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

pub fn add_environment(
  mut commands: Commands,
  env: Res<Environment>,
  mut meshes: ResMut<Assets<Mesh>>,
  mut materials: ResMut<Assets<StandardMaterial>>,
) {
  let l = env.transformed_l();
  // Need 6 rectangles, 2 for each boundary of the environment.
  // For the boundary along x, the rectangle should have width l.y and height l.z
  // And should be centred at (l.x / 2, 0, 0) and (-l.x / 2, 0, 0)
  // And so on.
  // The plane is on the XY plane, so we must do some rotations:
  // - For the planes along the x-direction, it should sit on the YZ plane, so rotate by 90 degrees around Z.
  let axis_configs = [
      (
          Vec2 {
              x: l.z as f32,
              y: l.y as f32,
          },
          Vec3::X,
          l.x,
          env.boundaries.0.x.closed,
          Vec3::Y,
      ),
      (
          Vec2 {
              x: l.z as f32,
              y: l.x as f32,
          },
          Vec3::Y,
          l.y,
          env.boundaries.0.y.closed,
          Vec3::X,
      ),
      (
          Vec2 {
              x: l.x as f32,
              y: l.y as f32,
          },
          Vec3::Z,
          l.z,
          env.boundaries.0.y.closed,
          Vec3::Y,
      ),
  ];

  for (size, axis, axis_l, closed, up) in axis_configs {
      for sgn in [1, -1] {
          commands.spawn((PbrBundle {
              mesh: meshes.add(shape::Quad { size, flip: false }.into()),
              material: materials.add(StandardMaterial::from(if closed {
                  Color::WHITE
              } else {
                  Color::YELLOW
              })),
              transform: Transform::from_translation(axis * (sgn as f32) * axis_l / 2.0 as f32)
                  .looking_to(axis * sgn as f32, up),
              ..default()
          },));
      }
  }
}
