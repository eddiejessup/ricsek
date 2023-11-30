use bevy::{prelude::*, render::view::visibility::Visibility};
use nalgebra::Vector3;

use crate::view::common::vec3_to_gvec3;

use super::common::spawn_arrow;

#[derive(Component, Clone)]
pub struct VectorLabel(pub String);

#[derive(Component, Clone)]
pub struct VectorSet(pub Vec<(VectorLabel, Vector3<f64>)>);

#[derive(Component)]
pub struct FlowVector;

#[derive(Resource)]
pub struct FlowViewState {
    pub vector_statuses: [bool; 9],
    pub max_arrow_length: f64,
    pub threshold: f64,
}

impl FlowViewState {
    pub fn new() -> Self {
        Self {
            vector_statuses: [false; 9],
            threshold: 0.0,
            max_arrow_length: 1.0,
        }
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
        "",
        TextStyle {
            font: asset_server.load("fonts/Inconsolata-Regular.ttf"),
            font_size: 40.0,
            color: Color::PURPLE,
        },
    ));

    // let mut n_vsets = 0;
    for vset_entity in q_vsets.iter() {
        // Create a new material here, because we will in fact change the color
        // of the flow vector.
        let child_material: Handle<StandardMaterial> =
            materials.add(StandardMaterial::from(Color::PURPLE.with_a(0.9)));

        let flow_vector_parent = commands
            .spawn((FlowVector, SpatialBundle { ..default() }))
            .with_children(|parent| {
                spawn_arrow(parent, &mut meshes, child_material);
            })
            .id();

        commands.entity(vset_entity).add_child(flow_vector_parent);

        // n_vsets += 1;
    }
    // info!("Added {} flow vector sets.", n_vsets);
}

pub fn update_flow(
    flow_view_state: Res<FlowViewState>,
    q_vectorset: Query<(&VectorSet, &Children)>,
    mut q_flow_vectors: Query<(&mut Transform, &mut Visibility), With<FlowVector>>,
    mut q_text: Query<&mut Text>,
) {
    let net_vs_and_labels: Vec<(Vec<VectorLabel>, Vector3<f64>)> = q_vectorset
        .iter()
        .map(|(vset, _children)| {
            // Keep the set of vectors that are enabled according to flow_view_state.
            let vset_enabled: Vec<(VectorLabel, Vector3<f64>)> = vset
                .0
                .iter()
                .enumerate()
                .filter_map(|(i, x)| {
                    if flow_view_state.vector_statuses[i] {
                        Some(x.clone())
                    } else {
                        None
                    }
                })
                .collect();

            let labels_enabled = vset_enabled
                .iter()
                .map(|(label, _)| label.clone())
                .collect();
            let net_v = vset_enabled.iter().map(|(_, v)| v).sum();

            (labels_enabled, net_v)
        })
        .collect();

    let mut text = q_text.get_single_mut().unwrap();
    text.sections[0].value = format!(
        "Flow vector: {}",
        match net_vs_and_labels.get(0) {
            Some((labels, _)) => labels
                .iter()
                .map(|s| s.clone().0)
                .collect::<Vec<String>>()
                .join(", "),
            None => "None".to_string(),
        }
    );

    let v_max_scale = 1.0;

    // Iterate over markers.
    // let mut n_markers = 0;
    for (i, (_vs, vset_children)) in q_vectorset.iter().enumerate() {
        // Get the selected flow vector.
        let v = net_vs_and_labels[i].1;

        // Map the flow vector to a magnitude in [0.0, 1.0].
        let mag_scale = (v.magnitude() / v_max_scale).min(1.0);

        // Set the scale of the arrow somewhere between 0 and env.arrow_length based on mag_scale value.
        let arrow_length = mag_scale * flow_view_state.max_arrow_length;
        // info!(
        //     "Magnitude scale: {}, Max arrow length: {}, Arrow scale: {}",
        //     mag_scale, flow_view_state.max_arrow_length, arrow_length
        // );

        for &vset_child in vset_children.iter() {
            let (mut transform, mut visibility) = match q_flow_vectors.get_mut(vset_child) {
                Ok(x) => x,
                Err(_) => continue,
            };

            // If the flow vector is too small, hide it.
            *visibility = if mag_scale < flow_view_state.threshold {
                // warn!(
                //     "Flow vector is below threshold {}, hiding it.",
                //     flow_view_state.threshold
                // );
                Visibility::Hidden
            } else {
                Visibility::Visible
            };

            // Set the orientation of the overall flow-vector.
            match vec3_to_gvec3(&v).try_normalize() {
                Some(glam_u) => {
                    if arrow_length > 1e-2 {
                        // info!("Setting arrow length to {}", arrow_length);
                        *transform = Transform::from_scale(Vec3::splat(arrow_length as f32))
                            .looking_to(glam_u, Vec3::Z)
                    } else {
                        // warn!("Arrow length is very small, hiding it.");
                        *visibility = Visibility::Hidden;
                    }
                }
                None => {
                    *visibility = Visibility::Hidden;
                    // warn!("Flow vector is zero, hiding it.");
                }
            };
        }

        // n_markers += 1;
    }
    // info!("Updated {} flow vectors.", n_markers);
}

pub fn change_view(keys: Res<Input<KeyCode>>, mut view_state: ResMut<FlowViewState>) {
    let entries = [
        (KeyCode::Key1, 0),
        (KeyCode::Key2, 1),
        (KeyCode::Key3, 2),
        (KeyCode::Key4, 3),
        (KeyCode::Key5, 4),
        (KeyCode::Key6, 5),
        (KeyCode::Key7, 6),
        (KeyCode::Key8, 7),
        (KeyCode::Key9, 8),
    ];
    for (keycode, ix) in entries {
        if keys.just_pressed(keycode) {
            view_state.vector_statuses[ix] = !view_state.vector_statuses[ix];
        }
    }
    // If t/shift+t is pressed, bump the threshold up/down.
    if keys.just_pressed(KeyCode::T) {
        view_state.threshold += 0.1
            * if keys.any_pressed([KeyCode::ShiftLeft, KeyCode::ShiftRight]) {
                -1.0
            } else {
                1.0
            };
        view_state.threshold = view_state.threshold.max(0.0);
    }
    // If l/shift+l is pressed, bump the arrow length up/down.
    if keys.just_pressed(KeyCode::L) {
        view_state.max_arrow_length *= 1.1_f64.powi(
            if keys.any_pressed([KeyCode::ShiftLeft, KeyCode::ShiftRight]) {
                -1
            } else {
                1
            },
        );
    }
}
