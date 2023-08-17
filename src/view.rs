pub mod common;
pub mod environment;
pub mod flow;
pub mod pan_orbit_camera;

use crate::{config::setup::SetupConfig, state::SimState};
use bevy::prelude::*;

// pub const TIME_STEP: f64 = 1.0 / 16.0;
pub const TIME_STEP: f64 = 1.0 / 160.0;

#[derive(Component)]
pub struct AgentId(pub usize);

#[derive(Component)]
pub struct AgentDirectionId(pub usize);

#[derive(Component)]
pub struct Movable;

#[derive(Resource)]
pub struct ViewState {
    pub i: usize,
    pub stepsize: usize,
}

impl Default for ViewState {
    fn default() -> Self {
        Self { i: 0, stepsize: 1 }
    }
}

impl ViewState {}

#[derive(Resource)]
pub struct SimStates(pub Vec<SimState>);

#[derive(Resource)]
pub struct SetupRes(pub SetupConfig);

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
