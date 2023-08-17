use std::f32::consts::FRAC_PI_2;

use bevy::{pbr::PbrBundle, prelude::*, render::view::visibility::Visibility};
use nalgebra::Vector3;

use crate::view::common::nalgebra_to_glam_vec;

use super::{common::increment_step, environment::Environment};

#[derive(Component)]
pub struct VectorSet(pub Vec<(String, Vector3<f64>)>);

#[derive(Component)]
pub struct FlowVectorMesh;

#[derive(Component)]
pub struct FlowVector;

#[derive(Resource)]
pub struct FlowViewState {
    pub i: usize,
    pub n_samples: usize,
}

impl FlowViewState {
    pub fn new(n_samples: usize) -> Self {
        Self { i: 0, n_samples }
    }
}

// Spawn flow vectors as children of each marker.
pub fn add_flow(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    q_vsets: Query<Entity, With<VectorSet>>,
) {
    commands.spawn(TextBundle::from_section(
        "Hello, Bevy!",
        TextStyle {
            font: asset_server.load("/Users/elliotmarsden/Library/Fonts/FiraCode-Medium.ttf"),
            font_size: 40.0,
            color: Color::PURPLE,
        },
    ));

    // Marker flow vectors.
    let cone: Handle<Mesh> = meshes.add(
        bevy_more_shapes::Cone {
            radius: 0.3,
            height: 0.4,
            segments: 32,
        }
        .into(),
    );
    let cylinder_height = 3.0;
    let cylinder: Handle<Mesh> = meshes.add(
        shape::Cylinder {
            radius: 0.1,
            height: cylinder_height,
            resolution: 32,
            ..default()
        }
        .into(),
    );
    let transform_mesh = Transform::from_rotation(Quat::from_rotation_x(-FRAC_PI_2));

    for vset_entity in q_vsets.iter() {
        // Create a new material here, because we will in fact change the color
        // of the flow vector.
        let child_material: Handle<StandardMaterial> =
            materials.add(StandardMaterial::from(Color::PURPLE));

        let flow_vector_parent = commands
            .spawn((FlowVector, SpatialBundle { ..default() }))
            .with_children(|parent| {
                parent.spawn((
                    PbrBundle {
                        mesh: cone.clone(),
                        material: child_material.clone(),
                        transform: transform_mesh.with_translation(-Vec3::Z * cylinder_height),
                        ..default()
                    },
                    FlowVectorMesh,
                ));
                parent.spawn((
                    PbrBundle {
                        mesh: cylinder.clone(),
                        material: child_material.clone(),
                        transform: transform_mesh
                            .with_translation(-Vec3::Z * cylinder_height / 2.0),
                        ..default()
                    },
                    FlowVectorMesh,
                ));
            })
            .id();

        commands.entity(vset_entity).add_child(flow_vector_parent);
    }
}

pub fn update_flow(
    env: Res<Environment>,
    flow_view_state: Res<FlowViewState>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    q_vectorset: Query<(&VectorSet, &Children)>,
    mut q_flow_vectors: Query<(&mut Transform, &mut Visibility, &Children), With<FlowVector>>,
    q_vec_flow_children: Query<&Handle<StandardMaterial>, With<FlowVectorMesh>>,
    mut q_text: Query<&mut Text>,
) {
    let all_xs: Vec<(String, Vector3<f64>)> = q_vectorset
        .iter()
        .filter_map(|(vset, _children)| vset.0.get(flow_view_state.i))
        .cloned()
        .collect();
    let may_label = match all_xs.get(0) {
        Some((label, _)) => Some(label),
        None => None,
    };
    let mut text = q_text.get_single_mut().unwrap();
    text.sections[0].value = match may_label {
        Some(label) => format!("Flow vector: {}", &label),
        None => "Flow vector: None".to_string(),
    };

    let all_vs: Vec<Vector3<f64>> = all_xs.iter().map(|(_, v)| v).cloned().collect();

    // Get maximum velocity magnitude.
    let (min_vel_log, max_vel_log) = all_vs.iter().map(|v| v.magnitude().log10()).fold(
        (f64::INFINITY, f64::NEG_INFINITY),
        |(mn, mx): (f64, f64), b| {
            (
                if b > f64::NEG_INFINITY { mn.min(b) } else { mn },
                mx.max(b),
            )
        },
    );

    let g = colorgrad::viridis();
    println!("Min velocity: {}", min_vel_log);
    println!("Max velocity: {}", max_vel_log);

    // Iterate over markers.
    for (vs, vset_children) in q_vectorset.iter() {
        // Get the selected flow vector.
        let (_label, v) = match vs.0.get(flow_view_state.i) {
            Some(x) => x,
            None => continue,
        };

        // Map the flow vector to a magnitude in [0.0, 1.0] normalized on the
        // max velocity, on a log scale.
        let mag_scale = (v.magnitude().log10() - min_vel_log) / (max_vel_log - min_vel_log);
        // Map normalized velocity to a point on the color spectrum.
        let mag_color = g.at(mag_scale);

        for &vset_child in vset_children.iter() {
            let (mut transform, mut visibility, flow_parent_children) =
                match q_flow_vectors.get_mut(vset_child) {
                    Ok(x) => x,
                    Err(_) => continue,
                };

            // If the flow vector is too small, hide it.
            *visibility = if mag_scale < 0.1 {
                Visibility::Hidden
            } else {
                Visibility::Visible
            };

            // Set the orientation of the overall flow-vector.
            match nalgebra_to_glam_vec(&v).try_normalize() {
                Some(glam_u) => {
                    *transform =
                        Transform::from_scale(Vec3::splat(env.transform_coord(env.arrow_length)))
                            .looking_to(glam_u, Vec3::Z)
                }
                None => {
                    *visibility = Visibility::Hidden;
                }
            };

            // Set the color of each part of the flow vector.
            for &flow_parent_child in flow_parent_children.iter() {
                let color_handle = q_vec_flow_children.get(flow_parent_child).unwrap();
                let color_mat = materials.get_mut(color_handle).unwrap();
                color_mat.base_color = Color::rgba(
                    mag_color.r as f32,
                    mag_color.g as f32,
                    mag_color.b as f32,
                    mag_color.a as f32,
                );
            }
        }
    }
    println!("");
    println!("");
}

pub fn change_view(keyboard_input: Res<Input<KeyCode>>, mut view_state: ResMut<FlowViewState>) {
    let backward = if keyboard_input.just_pressed(KeyCode::P) {
        Some(true)
    } else if keyboard_input.just_pressed(KeyCode::N) {
        Some(false)
    } else {
        None
    };

    if let Some(backward) = backward {
        let new_i = increment_step(view_state.i, backward, 100, 1);
        if new_i != view_state.i {
            println!("Changing view to {}", new_i);
            view_state.i = new_i;
        }
    }
}
