use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::prelude::*;

use super::environment::Environment;

// change input mapping for orbit and panning here
const ORBIT_BUTTON: MouseButton = MouseButton::Right;
const PAN_BUTTON: MouseButton = MouseButton::Middle;

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

pub fn pan_orbit_camera_update(
    mut ev_motion: EventReader<MouseMotion>,
    mut ev_scroll: EventReader<MouseWheel>,
    input_mouse: Res<ButtonInput<MouseButton>>,
    mut camera_query: Query<(&mut PanOrbitCamera, &mut Transform, &Projection)>,
    window_query: Query<&Window, With<bevy::window::PrimaryWindow>>,
) {
    let mut pan = Vec2::ZERO;
    let mut rotation_move = Vec2::ZERO;
    let mut scroll = 0.0;
    let mut orbit_button_changed = false;

    if input_mouse.pressed(ORBIT_BUTTON) {
        for ev in ev_motion.read() {
            debug!("Orbiting camera: {:?}", ev.delta);
            rotation_move += ev.delta;
        }
    } else if input_mouse.pressed(PAN_BUTTON) {
        // Pan only if we're not rotating at the moment
        for ev in ev_motion.read() {
            debug!("Panning camera: {:?}", ev.delta);
            pan += ev.delta;
        }
    }
    for ev in ev_scroll.read() {
        debug!("Scrolling camera: {:?}", ev.y);
        scroll += ev.y;
    }
    if input_mouse.just_released(ORBIT_BUTTON) || input_mouse.just_pressed(ORBIT_BUTTON) {
        debug!("Orbit button changed");
        orbit_button_changed = true;
    }

    for (mut pan_orbit, mut transform, projection) in camera_query.iter_mut() {
        if orbit_button_changed {
            // only check for upside down when orbiting started or ended this frame
            // if the camera is "upside" down, panning horizontally would be inverted, so invert the input to make it correct
            let up = transform.rotation * Vec3::Y;
            pan_orbit.upside_down = up.y <= 0.0;
            debug!("Camera upside down: {:?}", pan_orbit.upside_down);
        }

        let Ok(window) = window_query.get_single() else {
            warn!("No window found");
            return;
        };

        let mut any_change = false;
        if rotation_move.length_squared() > 0.0 {
            any_change = true;
            debug!("Rotating camera: {:?}", rotation_move);
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
            transform.rotation *= pitch; // rotate around local x axis
        } else if pan.length_squared() > 0.0 {
            any_change = true;
            debug!("Panning camera: {:?}", pan);
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
            any_change = true;
            debug!("Zooming camera: {:?}", scroll);
            pan_orbit.radius -= scroll * pan_orbit.radius * 0.01;
            // dont allow zoom to reach zero or you get stuck
            pan_orbit.radius = f32::max(pan_orbit.radius, 0.05);
        }

        if any_change {
            debug!("Updating camera position");
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
    Vec2::new(window.width(), window.height())
}

pub fn add_camera(mut commands: Commands, env: Res<Environment>) {
    let initial_z = match &env.boundaries {
        None => 10.0,
        Some(b) => b.l().max(),
    };

    let translation = Vec3::new(0.0, 0.0, initial_z as f32);
    // Log starting camera position
    info!("Starting camera position: {:?}", translation);
    let radius = translation.length();

    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(translation).looking_at(Vec3::ZERO, Vec3::Y),
        PanOrbitCamera {
            radius,
            ..Default::default()
        },
    ));

    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 100.0,
    });
}
