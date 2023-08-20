use std::time::Duration;

use bevy::{prelude::*, time::common_conditions::on_timer};
use nalgebra::{Point3, Vector3};
use ricsek::{
    config::setup::parameters::{
        common::{AxisBoundaryConfig, BoundaryConfig},
        singularities::{Singularity, SingularityParams},
    },
    view::{
        flow::{add_flow, update_flow, FlowViewState},
        pan_orbit_camera::{add_camera_startup, pan_orbit_camera_update},
        *,
    },
};

pub struct Marker {
    pub r: Point3<f64>,
    pub vs: flow::VectorSet,
}

#[derive(Resource)]
pub struct MarkerSet(pub Vec<Marker>);

// Helper to add markers just there to hold a vector-set.
pub fn add_flow_markers(
    mut commands: Commands,
    markers: Res<MarkerSet>,
    env: Res<environment::Environment>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Marker bases.
    let red: Handle<StandardMaterial> = materials.add(StandardMaterial::from(Color::RED));
    let cube: Handle<Mesh> = meshes.add((shape::Cube { size: 1.0 }).into()).into();

    for sample in markers.0.iter() {
        commands.spawn((
            PbrBundle {
                mesh: cube.clone(),
                material: red.clone(),
                transform: Transform::from_translation(env.transformed_vec3(sample.r)),
                ..default()
            },
            sample.vs.clone(),
        ));
    }
}

fn main() {
    let env = environment::Environment {
        boundaries: BoundaryConfig(Vector3::new(
            AxisBoundaryConfig {
                l: 200.0e-6,
                closed: true,
            },
            AxisBoundaryConfig {
                l: 200.0e-6,
                closed: true,
            },
            AxisBoundaryConfig {
                l: 100.0e-6,
                closed: true,
            },
        )),
        arrow_length: 3.0e-6,
        length_factor: 1e6,
    };

    let sample_rs: Vec<Point3<f64>> = ricsek::math::grid(env.boundaries.l(), 1000);

    let singularities: Vec<(String, Singularity)> = vec![
        (
            String::from("Stokeslet"),
            Singularity {
                point: Point3::origin(),
                params: SingularityParams::Stokeslet {
                    a: Vector3::new(1.0, 1.0, 0.0),
                },
            },
        ),
        (
            String::from("Stresslet"),
            Singularity {
                point: Point3::origin(),
                params: SingularityParams::Stresslet {
                    a: Vector3::new(1.0, 0.0, 0.0),
                    b: Vector3::new(0.0, 0.0, -1.0),
                },
            },
        ),
        (
            String::from("StokesDoublet"),
            Singularity {
                point: Point3::origin(),
                params: SingularityParams::StokesDoublet {
                    a: Vector3::new(1.0, 0.0, 0.0),
                    b: Vector3::new(0.0, 0.0, -1.0),
                },
            },
        ),
        (
            String::from("Rotlet"),
            Singularity {
                point: Point3::origin(),
                params: SingularityParams::Rotlet {
                    c: Vector3::new(0.0, 1.0, 0.0),
                },
            },
        ),
        (
            String::from("PotentialDoublet"),
            Singularity {
                point: Point3::origin(),
                params: SingularityParams::PotentialDoublet {
                    d: Vector3::new(0.0, 1.0, 0.0),
                },
            },
        ),
    ];

    let markers: Vec<Marker> = sample_rs
        .iter()
        .map(|r| Marker {
            r: *r,
            vs: flow::VectorSet(
                singularities
                    .iter()
                    .map(|(label, s)| (label.clone(), s.eval(*r)))
                    .collect(),
            ),
        })
        .collect();
    let n_samples = markers.len();

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(env)
        .insert_resource(MarkerSet(markers))
        .insert_resource(FlowViewState::new(n_samples))
        .add_systems(Startup, (add_flow_markers, add_camera_startup))
        .add_systems(PostStartup, add_flow)
        .add_systems(Startup, environment::add_environment)
        .add_systems(Update, pan_orbit_camera_update)
        .add_systems(
            Update,
            (
                flow::change_view.run_if(on_timer(Duration::from_secs_f64(TIME_STEP))),
                update_flow,
                bevy::window::close_on_esc,
            ),
        )
        .run();

    println!("Done!");
}
