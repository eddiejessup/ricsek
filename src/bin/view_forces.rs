use std::time::Duration;

use bevy::{prelude::*, sprite::MaterialMesh2dBundle, time::common_conditions::on_timer};

use nalgebra::{vector, zero, Point2, UnitVector2};
use ricsek::{
    dynamics::obstacle::{agent_obstacle_electro, agent_obstacle_hydro},
    math::{angle_to_x, linspace_grid},
    view::*,
};

#[derive(Resource)]
struct Samples(Vec<(Point2<f64>, UnitVector2<f64>)>);

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
        let base_pos = Vec2::new(transform_coord(r.x, l), transform_coord(r.y, l));

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
            mesh: meshes.add(arrow(0.7)).into(),
            material: materials.add(ColorMaterial::from(Color::GREEN)),
            transform: Transform::IDENTITY
                .with_translation(Vec3::new(base_pos.x, base_pos.y, 10.1))
                .with_rotation(Quat::from_rotation_z(angle_to_x((*u).into_inner()) as f32))
                .with_scale(Vec3::splat(10.0)),
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
        let base_pos = Vec2::new(transform_coord(r.x, l), transform_coord(r.y, l));

        let f = sim_setup
            .capsules
            .iter()
            .map(|cap| agent_obstacle_electro(*r, cap, radius, coeff))
            .sum();

        // Force field.
        commands.spawn(MaterialMesh2dBundle {
            mesh: meshes.add(arrow(1.0)).into(),
            material: materials.add(ColorMaterial::from(Color::BLUE)),
            transform: Transform::IDENTITY
                .with_translation(Vec3::new(base_pos.x, base_pos.y, 2.0))
                .with_rotation(Quat::from_rotation_z(angle_to_x(f) as f32))
                .with_scale(Vec3::splat(f.magnitude() as f32)),
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
        let base_pos = Vec2::new(transform_coord(r.x, l), transform_coord(r.y, l));

        let (v, om) = sim_setup
            .capsules
            .iter()
            .map(|cap| agent_obstacle_hydro(*r, *u, cap, aspect_ratio, coeff))
            .fold((zero(), 0.0), |(v, om), (v1, om1)| (v + v1, om + om1));

        // Velocity field.
        commands.spawn(MaterialMesh2dBundle {
            mesh: meshes.add(arrow(1.0)).into(),
            material: materials.add(ColorMaterial::from(Color::BLUE)),
            transform: Transform::IDENTITY
                .with_translation(Vec3::new(base_pos.x, base_pos.y, 2.0))
                .with_rotation(Quat::from_rotation_z(angle_to_x(v) as f32))
                .with_scale(Vec3::splat(v.magnitude().ln() as f32)),
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

    let u = UnitVector2::new_normalize(vector!(0.9, -0.435));

    let rs = linspace_grid(-sim_setup.params.l, sim_setup.params.l, 100);
    let samples = Samples(rs.iter().map(|r| (*r, u)).collect());

    // let samples = Samples(vec![
    //   (geo::Point::new(-0.000006000778198242187, 0.00004634765014648438), u),
    // ]);

    let env = EnvironmentRes {
        l: sim_setup.params.l,
    };

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(SimSetupRes(sim_setup))
        .insert_resource(env)
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
