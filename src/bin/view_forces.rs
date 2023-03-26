use std::{iter::Sum, time::Duration};

use bevy::{prelude::*, sprite::MaterialMesh2dBundle, time::common_conditions::on_timer};
use geo::Point;
use ricsek::{
    dynamics::obstacle::{agent_obstacle_electro, agent_obstacle_hydro},
    math::point::{array_angle_to_x, point_magnitude},
    view::*,
};

struct PointSum(geo::Point);

impl Sum<PointSum> for PointSum {
    fn sum<I>(iter: I) -> Self
    where
        I: Iterator<Item = PointSum>,
    {
        let mut sum = geo::Point::new(0.0, 0.0);
        for p in iter {
            sum += p.0;
        }
        PointSum(sum)
    }
}

#[derive(Resource)]
struct Samples(Vec<(Point, Point)>);

fn linspace(start: f64, stop: f64, n: usize) -> Vec<f64> {
    let step = (stop - start) / (n - 1) as f64;
    (0..n).map(|i| start + i as f64 * step).collect()
}

fn linspace_grid(start: f64, stop: f64, n: usize) -> Vec<Point> {
    let xs = linspace(start, stop, n);
    xs.iter()
        .flat_map(|x| xs.iter().map(move |y| Point::new(*x, *y)))
        .collect()
}

fn add_samples(
    mut commands: Commands,
    sim_setup: Res<SimSetupRes>,
    samples: Res<Samples>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let sim_setup = &sim_setup.0;

    let l = sim_setup.params.l;

    let radius = sim_setup.params.ag_radius;

    for (r, u) in &samples.0 {
        let base_pos = Vec2::new(transform_coord(r.x(), l), transform_coord(r.y(), l));

        // Agent shape (approximated as a circle).
        commands.spawn(MaterialMesh2dBundle {
            mesh: meshes
                .add(
                    (shape::Circle {
                        radius: transform_coord(radius, l),
                        vertices: 10,
                    })
                    .into(),
                )
                .into(),
            material: materials.add(ColorMaterial::from(Color::RED)),
            transform: Transform::IDENTITY.with_translation(Vec3::new(base_pos.x, base_pos.y, 1.0)),
            ..default()
        });

        // Agent direction.
        commands.spawn(MaterialMesh2dBundle {
            mesh: meshes.add(arrow(10.0, 0.7)).into(),
            material: materials.add(ColorMaterial::from(Color::GREEN)),
            transform: Transform::IDENTITY
                .with_translation(Vec3::new(base_pos.x, base_pos.y, 10.1))
                .with_rotation(Quat::from_rotation_z(array_angle_to_x(u.0) as f32)),
            ..default()
        });
    }
}

fn add_electro_forces(
    mut commands: Commands,
    sim_setup: Res<SimSetupRes>,
    samples: Res<Samples>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let sim_setup = &sim_setup.0;

    let l = sim_setup.params.l;

    let radius = sim_setup.params.ag_radius;
    let coeff = 10000000.0;

    for (r, _u) in &samples.0 {
        let base_pos = Vec2::new(transform_coord(r.x(), l), transform_coord(r.y(), l));

        let f = sim_setup
            .capsules
            .iter()
            .map(|cap| agent_obstacle_electro(*r, cap, radius, coeff))
            .map(PointSum)
            .sum::<PointSum>()
            .0;

        // Force field.
        commands.spawn(MaterialMesh2dBundle {
            mesh: meshes.add(arrow(point_magnitude(f) as f32, 1.0)).into(),
            material: materials.add(ColorMaterial::from(Color::BLUE)),
            transform: Transform::IDENTITY
                .with_translation(Vec3::new(base_pos.x, base_pos.y, 2.0))
                .with_rotation(Quat::from_rotation_z(array_angle_to_x(f.0) as f32)),
            ..default()
        });
    }
}

fn add_hydro_forces(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    sim_setup: Res<SimSetupRes>,
    samples: Res<Samples>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let sim_setup = &sim_setup.0;

    let l = sim_setup.params.l;

    let aspect_ratio = 1.0;
    let coeff = 1.0e-6;

    for (r, u) in &samples.0 {
        let base_pos = Vec2::new(transform_coord(r.x(), l), transform_coord(r.y(), l));

        let (v, om) = sim_setup
            .capsules
            .iter()
            .map(|cap| agent_obstacle_hydro(*r, *u, cap, aspect_ratio, coeff))
            .fold((geo::Point::new(0.0, 0.0), 0.0), |(v, om), (v1, om1)| {
                (v + v1, om + om1)
            });

        // Velocity field.
        commands.spawn(MaterialMesh2dBundle {
            mesh: meshes
                .add(arrow(point_magnitude(v).ln() as f32, 1.0))
                .into(),
            material: materials.add(ColorMaterial::from(Color::BLUE)),
            transform: Transform::IDENTITY
                .with_translation(Vec3::new(base_pos.x, base_pos.y, 2.0))
                .with_rotation(Quat::from_rotation_z(array_angle_to_x(v.0) as f32)),
            ..default()
        });

        // Torque field (just a sign indicator).
        commands.spawn(Text2dBundle {
            text: Text::from_section(
                if om > 0.0 { "+" } else { "-" },
                TextStyle {
                    font: asset_server.load("fonts/FiraCode-Regular.ttf"),
                    font_size: 20.0,
                    color: Color::WHITE,
                },
            ) // You can still add an alignment.
            .with_alignment(TextAlignment::Center),
            transform: Transform::IDENTITY.with_translation(Vec3::new(base_pos.x, base_pos.y, 3.0)),
            ..default()
        });
    }
}

fn main() {
    let conn = &mut ricsek::db::establish_connection();

    let run_id = ricsek::db::read_latest_run_id(conn);
    let sim_setup = ricsek::db::read_run(conn, run_id);

    let u = geo::Point::new(0.9, -0.435);

    let rs = linspace_grid(-sim_setup.params.l, sim_setup.params.l, 100);
    let samples = Samples(rs.iter().map(|r| (*r, u)).collect());

    // let samples = Samples(vec![
    //   (geo::Point::new(-0.000006000778198242187, 0.00004634765014648438), u),
    // ]);

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(SimSetupRes(sim_setup))
        .insert_resource(samples)
        .add_startup_system(add_obstacles)
        .add_startup_system(add_samples)
        .add_startup_system(add_electro_forces)
        .add_startup_system(add_hydro_forces)
        .add_startup_system(add_camera)
        .add_system(bevy::window::close_on_esc)
        .add_system(cursor_system.run_if(on_timer(Duration::from_secs_f64(TIME_STEP))))
        .run();

    println!("Done!");
}
