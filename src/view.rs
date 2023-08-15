pub mod environment;
pub mod pan_orbit_camera;

use std::f32::consts::FRAC_PI_2;

use crate::{config::setup::SetupConfig, state::SimState};
use bevy::{pbr::PbrBundle, prelude::*};

use self::environment::Environment;

// pub const TIME_STEP: f64 = 1.0 / 16.0;
pub const TIME_STEP: f64 = 1.0 / 160.0;

#[derive(Component)]
pub struct AgentId(pub usize);

#[derive(Component)]
pub struct SampleId(pub usize);

#[derive(Component)]
pub struct SingularityComp;

#[derive(Component)]
pub struct FlowVectorId;

#[derive(Component)]
pub struct AgentDirectionId(pub usize);

#[derive(Component)]
pub struct Movable;

#[derive(Resource)]
pub struct ViewState {
    pub i: usize,
}
impl ViewState {
    pub fn new() -> Self {
        Self { i: 0 }
    }
}

#[derive(Resource)]
pub struct SimStates(pub Vec<SimState>);

#[derive(Resource)]
pub struct SetupRes(pub SetupConfig);

pub fn add_agents(
    mut commands: Commands,
    sim_states: Res<SimStates>,
    env: Res<environment::Environment>,
    config: Res<SetupRes>,
    view_state: Res<ViewState>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let view_i = view_state.i;
    let cur_sim_state = &sim_states.0[view_i];

    let config = &config.0;

    let radius = config.parameters.sim_params.agent_radius;

    let materials = [
        materials.add(StandardMaterial::from(Color::RED)),
        materials.add(StandardMaterial::from(Color::GREEN)),
        materials.add(StandardMaterial::from(Color::BLUE)),
        materials.add(StandardMaterial::from(Color::PURPLE)),
        materials.add(StandardMaterial::from(Color::CYAN)),
    ];

    let agent_mesh = meshes.add(
        shape::UVSphere {
            radius: env.transform_coord(radius),
            ..default()
        }
        // shape::Capsule {
        //     radius: env.transform_coord(radius),
        //     depth: env.transform_coord(radius),
        //     ..default()
        // }
        .into(),
    );
    let arrow: Handle<Mesh> = meshes.add(
        bevy_more_shapes::Cone {
            radius: 1.0,
            height: 1.0,
            segments: 32,
        }
        .into(),
    );
    let cylinder_height = 3.0;
    let cylinder: Handle<Mesh> = meshes.add(
        shape::Cylinder {
            radius: 0.3,
            height: cylinder_height,
            resolution: 32,
            ..default()
        }
        .into(),
    );

    cur_sim_state.agents.iter().enumerate().for_each(|(i, _a)| {
        let material = materials[i % 5].clone();
        commands
            .spawn((AgentId(i), SpatialBundle::default()))
            .with_children(|parent| {
                parent.spawn(PbrBundle {
                    mesh: arrow.clone(),
                    material: material.clone(),
                    transform: Transform::IDENTITY
                        .with_rotation(Quat::from_rotation_x(-FRAC_PI_2))
                        .with_translation(-Vec3::Z * cylinder_height),
                    ..default()
                });
                parent.spawn(PbrBundle {
                    mesh: cylinder.clone(),
                    material: material.clone(),
                    transform: Transform::IDENTITY
                        .with_rotation(Quat::from_rotation_x(-FRAC_PI_2))
                        .with_translation(-Vec3::Z * cylinder_height / 2.0),
                    ..default()
                });
                parent.spawn(PbrBundle {
                    mesh: agent_mesh.clone(),
                    material: material.clone(),
                    // By default the capsule's cylinder points along +y.
                    // I want it to point along +z.
                    transform: Transform::IDENTITY.with_rotation(Quat::from_rotation_x(-FRAC_PI_2)),
                    ..default()
                });
            });
    });
}

pub fn cursor_system(
    // need to get window dimensions
    window_query: Query<&Window, With<bevy::window::PrimaryWindow>>,
    // query to get camera transform
    camera_q: Query<(&Camera, &GlobalTransform)>,
) {
    // get the camera info and transform
    // assuming there is exactly one main camera entity, so query::single() is OK
    let (camera, camera_transform) = camera_q.single();

    let Ok(window) = window_query.get_single() else {
      return;
    };

    let camera_loc = camera_transform.translation();

    eprintln!(
        "Camera loc: {}/{}/{}",
        camera_loc.x, camera_loc.y, camera_loc.z
    );

    // check if the cursor is inside the window and get its position
    // then, ask bevy to convert into world coordinates, and truncate to discard Z
    if let Some(world_position) = window
        .cursor_position()
        .and_then(|cursor| camera.viewport_to_world(camera_transform, cursor))
        .map(|ray| ray.origin)
    {
        eprintln!(
            "Pixel coords: {}/{}/{}",
            world_position.x, world_position.y, world_position.z
        );
        eprintln!(
            "Sim coords: {}/{}/{}",
            world_position.x, world_position.y, world_position.z
        );
    }
}

pub fn increment_step(cur_i: usize, backward: bool, maxi: usize) -> usize {
    if backward {
        match cur_i {
            0 => 0,
            _ => cur_i - 1,
        }
    } else {
        (cur_i + 1).min(maxi)
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
                x: l.y as f32,
                y: l.x as f32,
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
