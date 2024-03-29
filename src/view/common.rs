use std::f32::consts::FRAC_PI_2;

use bevy::prelude::*;
use nalgebra::{Vector3, Point3};

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
    let cone: Handle<Mesh> = meshes.add(
        bevy_more_shapes::Cone {
            radius: 1.0,
            height: 1.0,
            segments: 16,
        }
        .into(),
    );
    let cylinder_height = 3.0;
    let cylinder: Handle<Mesh> = meshes.add(
        shape::Cylinder {
            radius: 0.3,
            height: cylinder_height,
            resolution: 16,
            ..default()
        }
        .into(),
    );

    parent.spawn((
        PbrBundle {
            mesh: cone.clone(),
            material: material.clone(),
            // The cone's origin is at its base, so translate it up so the base
            // sits at the end of the cylinder.
            transform: transform_mesh.with_translation(-Vec3::Z * cylinder_height),
            ..default()
        },
        ArrowMesh,
    ));
    parent.spawn((
        PbrBundle {
            mesh: cylinder.clone(),
            material: material.clone(),
            // The cylinder's origin is at its centre, so translate it forward
            // by half its length to make its origin be its base.
            transform: transform_mesh.with_translation(-Vec3::Z * cylinder_height / 2.0),
            ..default()
        },
        ArrowMesh,
    ));
}

pub fn add_axis_arrows(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for (ax, color) in [
        (Vec3::X, Color::RED),
        (Vec3::Y, Color::GREEN),
        (Vec3::Z, Color::BLUE),
    ] {
        commands
            .spawn(SpatialBundle::from_transform(
                Transform::default()
                    .with_translation(Vec3 { x: 10.0, y: 10.0, z: 10.0 })
                    .looking_to(ax, Vec3::Y)
                    .with_scale(Vec3::splat(1.0)),
            ))
            .with_children(|parent| {
                spawn_arrow(
                    parent,
                    &mut meshes,
                    materials.add(StandardMaterial::from(color)),
                );
            });
    }
}
