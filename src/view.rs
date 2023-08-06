use crate::{
    config::setup::SetupConfig,
    math::capsule::Capsule,
    state::{Agent, SimState},
};
use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::{pbr::PbrBundle, prelude::*};
use nalgebra::{Point3, Vector3};

#[derive(Component)]
pub struct PanOrbitCamera {
    /// The "focus point" to orbit around. It is automatically updated when panning the camera
    pub focus: Vec3,
    pub radius: f32,
    pub upside_down: bool,
}

impl Default for PanOrbitCamera {
    fn default() -> Self {
        PanOrbitCamera {
            focus: Vec3::ZERO,
            radius: 5.0,
            upside_down: false,
        }
    }
}

pub fn pan_orbit_camera(
    mut ev_motion: EventReader<MouseMotion>,
    mut ev_scroll: EventReader<MouseWheel>,
    input_mouse: Res<Input<MouseButton>>,
    mut camera_query: Query<(&mut PanOrbitCamera, &mut Transform, &Projection)>,
    window_query: Query<&Window, With<bevy::window::PrimaryWindow>>,
) {
    // change input mapping for orbit and panning here
    let orbit_button = MouseButton::Right;
    let pan_button = MouseButton::Middle;

    let mut pan = Vec2::ZERO;
    let mut rotation_move = Vec2::ZERO;
    let mut scroll = 0.0;
    let mut orbit_button_changed = false;

    if input_mouse.pressed(orbit_button) {
        for ev in ev_motion.iter() {
            rotation_move += ev.delta;
        }
    } else if input_mouse.pressed(pan_button) {
        // Pan only if we're not rotating at the moment
        for ev in ev_motion.iter() {
            pan += ev.delta;
        }
    }
    for ev in ev_scroll.iter() {
        scroll += ev.y;
    }
    if input_mouse.just_released(orbit_button) || input_mouse.just_pressed(orbit_button) {
        orbit_button_changed = true;
    }

    for (mut pan_orbit, mut transform, projection) in camera_query.iter_mut() {
        if orbit_button_changed {
            // only check for upside down when orbiting started or ended this frame
            // if the camera is "upside" down, panning horizontally would be inverted, so invert the input to make it correct
            let up = transform.rotation * Vec3::Y;
            pan_orbit.upside_down = up.y <= 0.0;
        }

        let Ok(window) = window_query.get_single() else {
            return;
          };

        let mut any = false;
        if rotation_move.length_squared() > 0.0 {
            any = true;
            let window = get_window_size(window);
            let delta_x = {
                let delta = rotation_move.x / window.x * std::f32::consts::PI * 2.0;
                if pan_orbit.upside_down {
                    -delta
                } else {
                    delta
                }
            };
            let delta_y = rotation_move.y / window.y * std::f32::consts::PI;
            let yaw = Quat::from_rotation_y(-delta_x);
            let pitch = Quat::from_rotation_x(-delta_y);
            transform.rotation = yaw * transform.rotation; // rotate around global y axis
            transform.rotation = transform.rotation * pitch; // rotate around local x axis
        } else if pan.length_squared() > 0.0 {
            any = true;
            // make panning distance independent of resolution and FOV,
            let window = get_window_size(window);
            if let Projection::Perspective(projection) = projection {
                pan *= Vec2::new(projection.fov * projection.aspect_ratio, projection.fov) / window;
            }
            // translate by local axes
            let right = transform.rotation * Vec3::X * -pan.x;
            let up = transform.rotation * Vec3::Y * pan.y;
            // make panning proportional to distance away from focus point
            let translation = (right + up) * pan_orbit.radius;
            pan_orbit.focus += translation;
        } else if scroll.abs() > 0.0 {
            any = true;
            pan_orbit.radius -= scroll * pan_orbit.radius * 0.2;
            // dont allow zoom to reach zero or you get stuck
            pan_orbit.radius = f32::max(pan_orbit.radius, 0.05);
        }

        if any {
            // emulating parent/child to make the yaw/y-axis rotation behave like a turntable
            // parent = x and y rotation
            // child = z-offset
            let rot_matrix = Mat3::from_quat(transform.rotation);
            transform.translation =
                pan_orbit.focus + rot_matrix.mul_vec3(Vec3::new(0.0, 0.0, pan_orbit.radius));
        }
    }

    // consume any remaining events, so they don't pile up if we don't need them
    // (and also to avoid Bevy warning us about not checking events every frame update)
    ev_motion.clear();
}

fn get_window_size(window: &Window) -> Vec2 {
    Vec2::new(window.width() as f32, window.height() as f32)
}

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

#[derive(Resource)]
pub struct Obstacles(pub Vec<Capsule>);

#[derive(Component)]
pub struct Movable;

#[derive(Resource)]
pub struct ViewState {
    pub i: usize,
    pub rendered_i: Option<usize>,
}
impl ViewState {
    pub fn new() -> Self {
        Self {
            i: 0,
            rendered_i: None,
        }
    }

    pub fn is_stale(&self) -> bool {
        match self.rendered_i {
            Some(i) => self.i != i,
            None => true,
        }
    }

    pub fn mark_fresh(&mut self) {
        self.rendered_i = Some(self.i);
    }
}

#[derive(Resource)]
pub struct EnvironmentRes {
    pub l: f64,
    pub arrow_length: f64,
}

impl EnvironmentRes {
    pub fn transform_coord(&self, sd: f64) -> f32 {
        (sd * 1e6) as f32
    }

    pub fn invert_coord(&self, sd: f32) -> f64 {
        (sd * 1e-6) as f64
    }

    pub fn transformed_vec3(&self, sd: Point3<f64>) -> Vec3 {
        let st = sd.map(|x| self.transform_coord(x));
        Vec3::new(st.x, st.y, st.z)
    }

    pub fn inverted_vec3(&self, sd: Vec3) -> Point3<f64> {
        Point3::new(
            self.invert_coord(sd.x),
            self.invert_coord(sd.y),
            self.invert_coord(sd.z),
        )
    }
}

#[derive(Resource)]
pub struct SimStates(pub Vec<SimState>);

#[derive(Resource)]
pub struct SetupRes(pub SetupConfig);

pub fn invert_coord(sd: f32, sl: f64, pl: f64) -> f64 {
    sl * (sd as f64) / pl
}

pub fn nalgebra_to_glam_vec(v: &Vector3<f64>) -> Vec3 {
    Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}

pub fn agent_transform(env: &EnvironmentRes, a: &Agent) -> Transform {
    Transform::IDENTITY
        .with_translation(env.transformed_vec3(a.r))
        .with_rotation(Quat::from_rotation_arc(Vec3::X, nalgebra_to_glam_vec(&a.u)))
}

pub fn add_camera(mut commands: Commands) {
    let translation = Vec3::new(0.0, 0.0, 400.0);
    let radius = translation.length();

    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_translation(translation).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        PanOrbitCamera {
            radius,
            ..Default::default()
        },
    ));
}

pub fn add_agents(
    mut commands: Commands,
    sim_states: Res<SimStates>,
    env: Res<EnvironmentRes>,
    config: Res<SetupRes>,
    view_state: Res<ViewState>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let view_i = view_state.i;
    let cur_sim_state = &sim_states.0[view_i];

    let config = &config.0;

    let radius = config.parameters.sim_params.agent_radius;

    cur_sim_state.agents.iter().enumerate().for_each(|(i, a)| {
        let color = match i % 5 {
            0 => Color::RED,
            1 => Color::GREEN,
            2 => Color::BLUE,
            3 => Color::PURPLE,
            4 => Color::CYAN,
            _ => Color::WHITE,
        };

        commands.spawn((
            PbrBundle {
                mesh: meshes
                    .add(
                        (shape::Circle {
                            radius: env.transform_coord(radius),
                            vertices: 10,
                        })
                        .into(),
                    )
                    .into(),
                material: materials.add(StandardMaterial::from(color)),
                transform: agent_transform(&env, a),
                ..default()
            },
            AgentId(i),
        ));

        // commands.spawn((
        //     PbrBundle {
        //         mesh: meshes.add(arrow(1.0)).into(),
        //         material: materials.add(StandardMaterial::from(Color::RED)),
        //         // Add small amount to 'z' translation to avoid overlap
        //         transform: agent_transform(&env, a).with_scale(Vec3::splat(10.0)),
        //         ..default()
        //     },
        //     AgentId(i),
        //     AgentDirectionId(i),
        // ));
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
