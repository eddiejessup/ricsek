use std::f32::consts::FRAC_PI_2;

use crate::{config::setup::SetupConfig, geometry::cone::Cone};
use bevy::{
    color::palettes::css::{self},
    prelude::*,
};
use nalgebra::{Point3, Vector3};

pub const F_COEFF: f64 = 1e6;
pub const TORQUE_COEFF: f64 = 1e6;
pub const V_COEFF: f64 = 1e-1;

// Resources.

#[derive(Resource)]
pub struct SetupConfigRes(pub SetupConfig);

pub fn close_on_esc(
    mut commands: Commands,
    focused_windows: Query<(Entity, &Window)>,
    input: Res<ButtonInput<KeyCode>>,
) {
    for (window, focus) in focused_windows.iter() {
        if !focus.focused {
            continue;
        }

        if input.just_pressed(KeyCode::Escape) {
            commands.entity(window).despawn();
        }
    }
}

pub fn vec3_to_gvec3(v: &Vector3<f64>) -> Vec3 {
    Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}

pub fn point3_to_gvec3(v: &Point3<f64>) -> Vec3 {
    vec3_to_gvec3(&v.coords)
}

pub fn increment_step(cur_i: usize, backward: bool, maxi: usize, stepsize: usize) -> usize {
    if backward {
        if cur_i > stepsize {
            cur_i - stepsize
        } else {
            0
        }
    } else {
        (cur_i + stepsize).min(maxi)
    }
}

#[derive(Component)]
pub struct ArrowMesh;

pub fn spawn_arrow(
    parent: &mut ChildBuilder,
    meshes: &mut ResMut<Assets<Mesh>>,
    material: Handle<StandardMaterial>,
) {
    let transform_mesh = Transform::from_rotation(Quat::from_rotation_x(-FRAC_PI_2));
    let cone: Handle<Mesh> = meshes.add(Cone {
        radius: 0.5,
        height: 0.5,
        segments: 16,
    });
    let cylinder_height = 2.0;
    let cylinder: Handle<Mesh> = meshes.add(
        Cylinder::new(0.3, cylinder_height)
            .mesh()
            .resolution(16)
            .build(),
    );

    parent.spawn(((
        Mesh3d(cone.clone()),
        MeshMaterial3d(material.clone()),
        // The cone's origin is at its base, so translate it up so the base
        // sits at the end of the cylinder.
        transform_mesh.with_translation(-Vec3::Z * cylinder_height),
        ArrowMesh,
    ),));
    parent.spawn((
        Mesh3d(cylinder.clone()),
        MeshMaterial3d(material.clone()),
        // The cylinder's origin is at its centre, so translate it forward
        // by half its length to make its origin be its base.
        transform_mesh.with_translation(-Vec3::Z * cylinder_height / 2.0),
        ArrowMesh,
    ));
}

pub fn add_axis_arrows(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for (ax, color) in [
        (Vec3::X, Color::from(css::RED)),
        (Vec3::Y, Color::from(css::GREEN)),
        (Vec3::Z, Color::from(css::BLUE)),
    ] {
        commands
            .spawn(
                Transform::default()
                    .with_translation(Vec3 {
                        x: 10.0,
                        y: 10.0,
                        z: 10.0,
                    })
                    .looking_to(ax, Vec3::Y)
                    .with_scale(Vec3::splat(1.0)),
            )
            .with_children(|parent| {
                spawn_arrow(
                    parent,
                    &mut meshes,
                    materials.add(StandardMaterial::from(color)),
                );
            });
    }
}
