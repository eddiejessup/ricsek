use std::time::Duration;

use bevy::{prelude::*, sprite::MaterialMesh2dBundle, time::common_conditions::on_timer};
use ricsek::{
    dynamics::stokes_solutions::*,
    math::{
        linspace,
        point::{array_angle_to_x, point_magnitude},
    },
    view::*,
};

const L: f64 = 1.0;

#[derive(Resource)]
struct Samples(Vec<f64>);

fn add_samples(
    mut commands: Commands,
    samples: Res<Samples>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    for x in &samples.0 {
        for y in &samples.0 {
            let r = geo::Point::new(*x, *y);
            let base_pos = Vec2::new(transform_coord(r.x(), L), transform_coord(r.y(), L));

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
                transform: Transform::IDENTITY
                    .with_translation(Vec3::new(base_pos.x, base_pos.y, 1.0)),
                ..default()
            });
        }
    }
}

fn add_flow(
    mut commands: Commands,
    samples: Res<Samples>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let f = geo::Point::new(1.0, 0.0);
    let fr: geo::Point<f64> = geo::Point::new(0.0, 0.0);

    // Force origin.
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
        transform: Transform::IDENTITY.with_translation(Vec3::new(
            fr.x() as f32,
            fr.y() as f32,
            1.0,
        )),
        ..default()
    });

    for x in &samples.0 {
        for y in &samples.0 {
            let pr = geo::Point::new(*x, *y);
            let base_pos = Vec2::new(transform_coord(pr.x(), L), transform_coord(pr.y(), L));

            // let v = stokeslet_u(f, fr, pr);
            // let v = doublet_u(f, fr, pr);
            // let v = couplet_u(1.0, fr, pr);
            let v = stresslet_u(geo::Point::new(1.0, -1.0), 0.0, fr, pr) * 10.0;

            // Velocity field.
            commands.spawn(MaterialMesh2dBundle {
                mesh: meshes.add(arrow(0.5)).into(),
                material: materials.add(ColorMaterial::from(Color::BLUE)),
                transform: Transform::IDENTITY
                    .with_translation(Vec3::new(base_pos.x, base_pos.y, 2.0))
                    .with_rotation(Quat::from_rotation_z(array_angle_to_x(v.0) as f32))
                    .with_scale(Vec3::splat(point_magnitude(v).min(30.0) as f32)),
                ..default()
            });
        }
    }
}

fn main() {
    let samples = Samples(linspace(-L, L, 120));
    let env = EnvironmentRes { l: L };

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(samples)
        .insert_resource(env)
        .add_startup_system(add_samples)
        .add_startup_system(add_flow)
        .add_startup_system(add_camera)
        .add_system(bevy::window::close_on_esc)
        .add_system(cursor_system.run_if(on_timer(Duration::from_secs_f64(TIME_STEP))))
        .run();

    println!("Done!");
}
