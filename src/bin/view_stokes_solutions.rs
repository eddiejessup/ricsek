use std::time::Duration;

use bevy::{prelude::*, sprite::MaterialMesh2dBundle, time::common_conditions::on_timer};
use nalgebra::{Point2, Vector2, point};
use ricsek::{
    dynamics::stokes_solutions::*,
    math::{angle_to_x, linspace, capsule::Capsule},
    view::*,
};

const L: f64 = 1.0;

#[derive(Resource)]
struct Samples(Vec<f64>);

enum SingularityParams {
    Stokeslet(Vector2<f64>),
    Doublet(Vector2<f64>),
    Rotlet(f64),
    Stresslet(Vector2<f64>, f64),
}

struct Singularity {
    point: Point2<f64>,
    params: SingularityParams,
}

impl Singularity {
    fn eval(&self, pr: Point2<f64>) -> Vector2<f64> {
        match self.params {
            SingularityParams::Stokeslet(f) => stokeslet_u(f, self.point, pr),
            SingularityParams::Doublet(f) => doublet_u(f, self.point, pr),
            SingularityParams::Rotlet(f) => rotlet_u(f, self.point, pr),
            SingularityParams::Stresslet(stress_diag, stress_off) => {
                stresslet_u(stress_diag, stress_off, self.point, pr)
            }
        }
    }
}

#[derive(Resource)]
struct SingularitySet(Vec<Singularity>);

fn add_samples(
    mut commands: Commands,
    samples: Res<Samples>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
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
                transform: Transform::IDENTITY.with_translation(transform_vec(r, L, 1.0)),
                ..default()
            });
        }
    }
}

fn add_flow(
    mut commands: Commands,
    samples: Res<Samples>,
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
            transform: Transform::IDENTITY.with_translation(transform_vec(s.point, L, 3.0)),
            ..default()
        });
    }

    for x in &samples.0 {
        for y in &samples.0 {
            let pr = Point2::new(*x, *y);
            let base_pos = Vec2::new(transform_coord(pr.x, L), transform_coord(pr.y, L));

            let v = singularities.0.iter().map(|s| s.eval(pr)).sum();

            // Velocity field.
            commands.spawn(MaterialMesh2dBundle {
                mesh: meshes.add(arrow(0.5)).into(),
                material: materials.add(ColorMaterial::from(Color::BLUE)),
                transform: Transform::IDENTITY
                    .with_translation(Vec3::new(base_pos.x, base_pos.y, 2.0))
                    .with_rotation(Quat::from_rotation_z(angle_to_x(v) as f32))
                    .with_scale(Vec3::splat(v.magnitude().min(30.0) as f32)),
                ..default()
            });
        }
    }
}

fn main() {
    let samples = Samples(linspace(-L, L, 120));
    let env = EnvironmentRes { l: L };

    // // Read force parameters from command line.
    // let force_params = command!() // requires `cargo` feature
    //   .arg(arg!([name] "Optional name to operate on"))
    //   .arg(
    //       arg!(
    //           -c --config <FILE> "Sets a custom config file"
    //       )
    //       // We don't have syntax yet for optional options, so manually calling `required`
    //       .required(false)
    //       .value_parser(value_parser!(PathBuf)),
    //   )
    //   .arg(arg!(
    //       -d --debug ... "Turn debugging information on"
    //   ))
    //   .subcommand(
    //       Command::new("test")
    //           .about("does testing things")
    //           .arg(arg!(-l --list "lists test values").action(ArgAction::SetTrue)),
    //   )
    //   .get_matches();

    let dir = 3.0 * Vector2::new(0.8, 0.2);
    let singularities = SingularitySet(vec![
        Singularity {
            point: Point2::new(-0.2, 0.0),
            params: SingularityParams::Stokeslet(dir),
        },
        Singularity {
            point: Point2::new(0.2, 0.0),
            params: SingularityParams::Stokeslet(-dir),
        },

        // Singularity {
        //     point: Point2::new(-0.3, 0.0),
        //     params: SingularityParams::Stresslet(Vector2::new(-10.0, 10.0), 10.0),
        // },
        // Singularity {
        //     point: Point2::new(0.3, 0.0),
        //     params: SingularityParams::Stresslet(-Vector2::new(-10.0, 10.0), 10.0),
        // },
        // Singularity {
        //     point: Point2::new(0.5, 0.0),
        //     params: SingularityParams::Stokeslet(Vector2::new(-1.0, 1.0)),
        // },
        // Singularity {
        //     point: Point2::new(-0.3, 0.3),
        //     params: SingularityParams::Doublet(Vector2::new(-0.5, 0.3)),
        // },
        // Singularity {
        //     point: Point2::new(-0.3, -0.3),
        //     params: SingularityParams::Rotlet(0.5),
        // },
    ]);

    let capsule_radius = 0.005;
    let capsules: Vec<Capsule> = vec![(point![0.0, -0.25], point![0.0, 0.25])]
        .iter()
        .map(|(s, e)| (s * L / 2.0, e * L / 2.0))
        .map(|(s, e)| Capsule::new(s, e, capsule_radius))
        .collect();

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(samples)
        .insert_resource(Obstacles(capsules))
        .insert_resource(singularities)
        .insert_resource(env)
        .add_startup_system(add_samples)
        .add_startup_system(add_flow)
        .add_startup_system(add_camera)
        .add_startup_system(add_obstacles)
        .add_system(bevy::window::close_on_esc)
        .add_system(cursor_system.run_if(on_timer(Duration::from_secs_f64(TIME_STEP))))
        .run();

    println!("Done!");
}
