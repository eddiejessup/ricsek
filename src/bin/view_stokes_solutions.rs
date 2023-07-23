use std::time::Duration;

use bevy::{prelude::*, sprite::MaterialMesh2dBundle, time::common_conditions::on_timer};
use nalgebra::{Point2, Vector2};
use ricsek::{
    dynamics::stokes_solutions::*,
    math::{angle_to_x, capsule::Capsule, linspace},
    view::*,
};

const L: f64 = 1.0;

#[derive(Resource)]
struct Samples(Vec<f64>);

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
        stress_diag: Vector2<f64>,
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
            } => stresslet_u(stress_diag, stress_off, r),
        }
    }
}

#[derive(Resource)]
struct SingularitySet(Vec<Singularity>);

fn add_samples(
    mut commands: Commands,
    samples: Res<Samples>,
    env: Res<EnvironmentRes>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    // Point as circle.
    for x in &samples.0 {
        for y in &samples.0 {
            let r = Point2::new(*x, *y);
            // Sample point (shown as a circle).
            commands.spawn(MaterialMesh2dBundle {
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
                transform: Transform::IDENTITY.with_translation(env.transformed_vec3(r, 1.0)),
                ..default()
            });
        }
    }
}

fn add_flow(
    mut commands: Commands,
    samples: Res<Samples>,
    env: Res<EnvironmentRes>,
    singularities: Res<SingularitySet>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    // Force origin.
    for s in &singularities.0 {
        commands.spawn(MaterialMesh2dBundle {
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
            transform: Transform::IDENTITY.with_translation(env.transformed_vec3(s.point, 3.0)),
            ..default()
        });
    }

    // Accumulate vector of (p, v) pairs.
    let mut sample_evals: Vec<(Point2<f64>, Vector2<f64>)> = Vec::new();
    for x in &samples.0 {
        for y in &samples.0 {
            let pr = Point2::new(*x, *y);
            let v = singularities.0.iter().map(|s| s.eval(pr)).sum();
            sample_evals.push((pr, v));
        }
    }

    // Get maximum velocity magnitude.
    let max_vel_log = sample_evals
        .iter()
        .map(|(_, v)| v.magnitude().log10())
        .fold(0.0, |a: f64, b: f64| a.max(b));

    let g = colorgrad::viridis();

    // Iterate over (p, v) pairs to draw them.
    for (p, v) in &sample_evals {
        // Get a magnitude in [0.0, 1.0] normalized on the max velocity, on a log scale.
        let vm_norm_log = (v.magnitude().log10() - max_vel_log) / -max_vel_log;

        // Map normalized velocity to a point on the color spectrum.
        let c_arr_64: [f64; 4] = g.at(vm_norm_log).to_array();
        let c_arr_32: [f32; 4] = [
            c_arr_64[0] as f32,
            c_arr_64[1] as f32,
            c_arr_64[2] as f32,
            c_arr_64[3] as f32,
        ];

        let base_pos = env.transformed_vec2(*p);
        commands.spawn(MaterialMesh2dBundle {
            mesh: meshes.add(arrow(0.5)).into(),
            material: materials.add(ColorMaterial::from(Color::from(c_arr_32))),
            transform: Transform::IDENTITY
                .with_translation(Vec3::new(base_pos.x, base_pos.y, 2.0))
                .with_rotation(Quat::from_rotation_z(angle_to_x(v) as f32))
                .with_scale(Vec3::splat(env.arrow_length_pixels as f32)),
            ..default()
        });
    }
}

fn main() {
    let env = EnvironmentRes {
        l: L,
        window_size: 1500.0,
        arrow_length_pixels: 10.0,
    };

    let n_arrows = (env.window_size / (2.0 * env.arrow_length_pixels)) as usize;

    let samples = Samples(linspace(-env.l / 2.0, env.l / 2.0, n_arrows));

    let singularities = SingularitySet(vec![
        // // Top-left corner: two stresslets making a stresslet 'manually'.
        // Singularity {
        //     point: Point2::new(-0.25, 0.25),
        //     params: SingularityParams::Stokeslet {
        //         force: 20.0 * Vector2::new(1.0, 0.0),
        //     },
        // },
        // Singularity {
        //     point: Point2::new(-0.25 + 0.03, 0.25),
        //     params: SingularityParams::Stokeslet {
        //         force: 20.0 * Vector2::new(-1.0, 0.0),
        //     },
        // },
        // Top-right corner: a stresslet
        Singularity {
            point: Point2::new(0.25, 0.25),
            params: SingularityParams::Stresslet {
                stress_diag: Vector2::new(1.0, -1.0),
                stress_off: 0.0,
            },
        },
        // // Bottom-left corner: a doublet
        // Singularity {
        //     point: Point2::new(-0.25, -0.25),
        //     params: SingularityParams::Doublet {
        //         strength: Vector2::new(1.0, 0.0),
        //     },
        // },
        // // Bottom-right corner: a rotlet
        // Singularity {
        //     point: Point2::new(0.25, -0.25),
        //     params: SingularityParams::Rotlet { torque: 1.0 },
        // },
    ]);

    let capsule_radius = 0.005;
    let capsules: Vec<Capsule> = vec![
        // (point![0.0, -0.25], point![0.0, 0.25])
    ]
    .iter()
    .map(|(s, e): &(Point2<f64>, Point2<f64>)| (s * L / 2.0, e * L / 2.0))
    .map(|(s, e)| Capsule::new(s, e, capsule_radius))
    .collect();

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(samples)
        .insert_resource(Obstacles(capsules))
        .insert_resource(singularities)
        .insert_resource(env)
        .add_systems(Startup, (add_samples, add_flow, add_camera, add_obstacles))
        .add_systems(
            Update,
            (
                bevy::window::close_on_esc,
                cursor_system.run_if(on_timer(Duration::from_secs_f64(TIME_STEP))),
            ),
        )
        .run();

    println!("Done!");
}
