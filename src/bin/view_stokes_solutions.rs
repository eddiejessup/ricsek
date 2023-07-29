use std::time::Duration;

use bevy::{prelude::*, sprite::MaterialMesh2dBundle, time::common_conditions::on_timer};
use nalgebra::{Point2, Vector2};
use ricsek::{
    dynamics::stokes_solutions::*,
    math::{angle_to_x, linspace},
    view::*,
};

const L: f64 = 1.0;

#[derive(Resource)]
struct Samples(Vec<Point2<f64>>);

#[derive(Debug)]
enum SingularityParams {
    Stokeslet {
        force: Vector2<f64>,
    },
    Doublet {
        strength: Vector2<f64>,
    },
    Rotlet {
        torque: f64,
    },
    Stresslet {
        stress_diag: f64,
        stress_off: f64,
    },
}

struct Singularity {
    point: Point2<f64>,
    params: SingularityParams,
}

impl Singularity {
    fn eval(&self, pr: Point2<f64>) -> Vector2<f64> {
        let r = pr - self.point;
        match self.params {
            SingularityParams::Stokeslet { force } => stokeslet_u(force, r),
            SingularityParams::Doublet { strength } => doublet_u(strength, r),
            SingularityParams::Rotlet { torque } => rotlet_u(torque, r),
            SingularityParams::Stresslet {
                stress_diag,
                stress_off,
            // } => stresslet_u(stress_diag, stress_off, r),
            // } => stresslet_u_2(nalgebra::vector!(stress_diag, 0.0), r),
            } => stresslet_u_2(nalgebra::vector!(stress_diag, 0.0), r) - stresslet_u(stress_diag, stress_off, r),
        }
    }
}

#[derive(Resource)]
struct Singularities(Vec<Singularity>);

fn add_samples(
    mut commands: Commands,
    samples: Res<Samples>,
    env: Res<EnvironmentRes>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    // Point as circle.
    for (i, r) in samples.0.iter().enumerate() {
        // Sample point (shown as a circle).
        commands.spawn((
            MaterialMesh2dBundle {
                mesh: meshes
                    .add(
                        (shape::Circle {
                            radius: 1.0,
                            vertices: 10,
                        })
                        .into(),
                    )
                    .into(),
                material: materials.add(ColorMaterial::from(Color::RED)),
                transform: Transform::IDENTITY.with_translation(env.transformed_vec3(*r, 1.0)),
                ..default()
            },
            SampleId(i),
        ));
    }
}

fn add_flow(
    mut commands: Commands,
    samples: Res<Samples>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    // Force origin.
    commands.spawn((
        MaterialMesh2dBundle {
            mesh: meshes
                .add(
                    (shape::Circle {
                        radius: 10.0,
                        vertices: 10,
                    })
                    .into(),
                )
                .into(),
            material: materials.add(ColorMaterial::from(Color::PURPLE)),
            ..default()
        },
        SingularityComp,
    ));

    // Sample flow vectors.
    for (i, _) in samples.0.iter().enumerate() {
        commands.spawn((
            MaterialMesh2dBundle {
                mesh: meshes.add(arrow(0.5)).into(),
                material: materials.add(ColorMaterial::from(Color::PURPLE)),
                ..default()
            },
            SampleId(i),
            FlowVectorId,
        ));
    }
}

fn update_flow(
    samples: Res<Samples>,
    env: Res<EnvironmentRes>,
    singularities: Res<Singularities>,
    view_state: Res<ViewState>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    // mut query_sing: Query<(&mut Transform, &SingularityComp)>,
    mut query_samp: Query<(
        &mut Transform,
        &Handle<ColorMaterial>,
        &SampleId,
        &FlowVectorId,
    )>,
) {
    let singularity = &singularities.0[view_state.i];

    // let (mut sing_transform, _comp) = query_sing.single_mut();
    // *sing_transform =
    //     Transform::IDENTITY.with_translation(env.transformed_vec3(singularity.point, 3.0));

    let vs: Vec<Vector2<f64>> = samples.0.iter().map(|r| singularity.eval(*r)).collect();

    // Get maximum velocity magnitude.
    let max_vel_log = vs
        .iter()
        .map(|v| v.magnitude().log10())
        .fold(0.0, |a: f64, b: f64| a.max(b));

    let g = colorgrad::viridis();
    println!("Max velocity: {}", max_vel_log);

    // Iterate over (p, v) pairs to draw them.
    for (mut transform, color_handle, sample_id, _flow) in query_samp.iter_mut() {
        let v = vs[sample_id.0];
        let r = samples.0[sample_id.0];
        // Get a magnitude in [0.0, 1.0] normalized on the max velocity, on a log scale.
        let vm_norm_log = (v.magnitude().log10() - max_vel_log) / -max_vel_log;

        if sample_id.0 == 0 {
            println!("Sample 0: v: {:?}, vm_norm_log: {}", v, vm_norm_log);
        }

        // Map normalized velocity to a point on the color spectrum.
        let c_arr_64: [f64; 4] = g.at(vm_norm_log).to_array();
        let c_arr_32: [f32; 4] = [
            c_arr_64[0] as f32,
            c_arr_64[1] as f32,
            c_arr_64[2] as f32,
            c_arr_64[3] as f32,
        ];

        if sample_id.0 == 0 {
            println!(
                "Sample 0: c_arr_64: {:?}, c_arr_32: {:?}",
                c_arr_64, c_arr_32
            );
        }

        let base_pos = env.transformed_vec2(r);
        *transform = Transform::IDENTITY
            .with_translation(Vec3::new(base_pos.x, base_pos.y, 2.0))
            .with_rotation(Quat::from_rotation_z(angle_to_x(&v) as f32))
            .with_scale(Vec3::splat(env.arrow_length_pixels as f32));

        let mut color_mat = materials.get_mut(color_handle).unwrap();
        color_mat.color = Color::from(c_arr_32);
    }
}

fn change_view(
    keyboard_input: Res<Input<KeyCode>>,
    mut view_state: ResMut<ViewState>,
    mut singularities: ResMut<Singularities>,
) {
    let backward = if keyboard_input.just_pressed(KeyCode::Left) {
        Some(true)
    } else if keyboard_input.just_pressed(KeyCode::Right) {
        Some(false)
    } else {
        None
    };

    if let Some(backward) = backward {
        let new_i = increment_step(view_state.i, backward, singularities.0.len() - 1);
        if new_i != view_state.i {
            println!("Changing view to {}", new_i);
            view_state.i = new_i;
        }
    }

    let mag_down = if keyboard_input.pressed(KeyCode::Down) {
        Some(true)
    } else if keyboard_input.pressed(KeyCode::Up) {
        Some(false)
    } else {
        None
    };
    let singularity = &mut singularities.0[view_state.i];

    if let Some(mag_down) = mag_down {
        adjust_singularity_params(&mut singularity.params, if mag_down { 1.1 } else { 0.9 });
        println!("Singularity params: {:?}", singularity.params);
    }
}

fn adjust_singularity_params(p: &mut SingularityParams, factor: f64) {
    match p {
        SingularityParams::Stokeslet { force } => {
            *force *= factor;
        }
        SingularityParams::Doublet { strength } => {
            *strength *= factor;
        }
        SingularityParams::Rotlet { torque } => {
            *torque *= factor;
        }
        SingularityParams::Stresslet { stress_diag, .. } => {
            *stress_diag *= factor;
        }
    }
}
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

fn main() {
    let env = EnvironmentRes {
        l: L,
        window_size: 1500.0,
        arrow_length_pixels: 10.0,
    };

    let n_arrows = (env.window_size / (2.0 * env.arrow_length_pixels)) as usize;

    let samples_1d = linspace(-env.l / 2.0, env.l / 2.0, n_arrows);

    let mut samples_2d: Vec<Point2<f64>> = Vec::new();
    for x in &samples_1d {
        for y in &samples_1d {
            samples_2d.push(Point2::new(*x, *y));
        }
    }
    let samples = Samples(samples_2d);

    let singularities = Singularities(vec![
        // Singularity {
        //     point: Point2::new(0.0, 0.0),
        //     params: SingularityParams::Stokeslet {
        //         force: 20.0 * Vector2::new(1.0, 0.0),
        //     },
        // },
        Singularity {
            point: Point2::new(0.0, 0.0),
            params: SingularityParams::Stresslet {
                stress_diag: 1.0,
                stress_off: 0.0,
            },
        },
        // Singularity {
        //     point: Point2::new(0.0, 0.0),
        //     params: SingularityParams::Stresslet {
        //         stress_diag: -1.0,
        //         stress_off: 0.0,
        //     },
        // },
        Singularity {
            point: Point2::new(0.0, 0.0),
            params: SingularityParams::Doublet {
                strength: Vector2::new(1.0, 0.0),
            },
        },
        // Singularity {
        //     point: Point2::new(0.0, -0.0),
        //     params: SingularityParams::Rotlet { torque: 1.0 },
        // },
    ]);

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(samples)
        .insert_resource(singularities)
        .insert_resource(env)
        .insert_resource(ViewState::new())
        .add_systems(Startup, (add_samples, add_flow, add_camera))
        .add_systems(
            Update,
            (
                change_view.run_if(on_timer(Duration::from_secs_f64(TIME_STEP))),
                update_flow.run_if(on_timer(Duration::from_secs_f64(TIME_STEP))),
                bevy::window::close_on_esc,
                cursor_system.run_if(on_timer(Duration::from_secs_f64(TIME_STEP))),
            ),
        )
        .run();

    println!("Done!");
}
