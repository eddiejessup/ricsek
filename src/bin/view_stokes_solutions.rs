use std::time::Duration;

use bevy::{prelude::*, time::common_conditions::on_timer};
use nalgebra::{Point3, Vector3};
use ricsek::{
    config::setup::parameters::{
        common::{AxisBoundaryConfig, BoundaryConfig},
        singularities::{Singularity, SingularityParams},
    },
    view::{
        common::{add_axis_arrows, point3_to_gvec3},
        flow::{add_flow, update_flow, FlowViewState},
        pan_orbit_camera::{add_camera, pan_orbit_camera_update},
        *,
    },
};

pub struct Marker {
    pub r: Point3<f64>,
    pub vs: flow::VectorSet,
}

#[derive(Resource)]
pub struct MarkerSet(pub Vec<Marker>);

#[derive(Resource)]
pub struct SingularitySet(pub Vec<(flow::VectorLabel, Singularity)>);

// Helper to add markers just there to hold a vector-set.
pub fn add_flow_markers(
    mut commands: Commands,
    markers: Res<MarkerSet>,
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
                transform: Transform::from_translation(point3_to_gvec3(&sample.r)),
                ..default()
            },
            sample.vs.clone(),
        ));
    }
}

pub fn add_singularities(
    mut commands: Commands,
    singularities: Res<SingularitySet>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Marker bases.
    let material_handle: Handle<StandardMaterial> =
        materials.add(StandardMaterial::from(Color::YELLOW));
    let mesh_handle: Handle<Mesh> = meshes.add((shape::Cube { size: 10.0 }).into()).into();

    for (label, sing) in singularities.0.iter() {
        commands.spawn((
            PbrBundle {
                mesh: mesh_handle.clone(),
                material: material_handle.clone(),
                transform: Transform::from_translation(point3_to_gvec3(&sing.point)),
                ..default()
            },
            label.clone(),
        ));
    }
}

fn stokeslet_mirror(
    p: Point3<f64>,
    d: Vector3<f64>,
    stokeslet_force: Vector3<f64>,
) -> Vec<Singularity> {
    let h = d.norm();
    [
        SingularityParams::Stokeslet {
            a: -stokeslet_force,
        },
        SingularityParams::StokesDoublet {
            a: 2.0 * h * stokeslet_force,
            b: -d.normalize(),
        },
        SingularityParams::PotentialDoublet {
            d: -2.0 * h.powi(2) * stokeslet_force,
        },
    ]
    .iter()
    .map(|params| Singularity {
        point: p + 2.0 * d,
        params: params.clone(),
    })
    .collect()
}

fn main() {
    env_logger::init();
    let bc = BoundaryConfig(Vector3::new(
        AxisBoundaryConfig {
            l: 200.0,
            closed: true,
        },
        AxisBoundaryConfig {
            l: 200.0,
            closed: true,
        },
        AxisBoundaryConfig {
            l: 200.0,
            closed: true,
        },
    ));
    let env = environment::Environment {
        boundaries: Some(bc.clone()),
        arrow_length: 2.0,
    };

    let mut sample_rs = Vec::new();
    // for p in ricsek::math::grid(1.2 * env.boundaries.l(), 2000) {
    //     sample_rs.push(p);
    // }

    let sample_l = nalgebra::Vector2::new(bc.l().x, bc.l().y);
    let sample_z = bc.l_half().z;
    let step = 10.0;
    for p in ricsek::geometry::grid_2d(sample_l, step) {
        sample_rs.push(Point3::new(p.x, p.y, sample_z));
        sample_rs.push(Point3::new(p.x, p.y, -sample_z));
    }

    let stokeslet_strength = 10.0e-5;
    let stokeslet_force = Vector3::new(0.0, stokeslet_strength, 0.0);
    let stokeslet_origin_point = Point3::origin();
    let stokeslet_origin = Singularity {
        point: stokeslet_origin_point,
        params: SingularityParams::Stokeslet { a: stokeslet_force },
    };
    let d = Vector3::new(0.0, 0.0, bc.0.z.l_half());

    // let stokeslet_mirror_point = stokeslet_origin.point + Vector3::new(0.0, 0.0, 2.0 * h);
    // let stokeslet_mirror = Singularity {
    //     point: stokeslet_mirror_point,
    //     params: SingularityParams::Stokeslet {
    //         a: -stokeslet_force,
    //     },
    // };
    // let mirror_stokes_doublet_strength = 2.0 * h * stokeslet_strength;
    // let stokes_doublet_mirror = Singularity {
    //     point: stokeslet_mirror_point,
    //     params: SingularityParams::StokesDoublet {
    //         a: Vector3::new(mirror_stokes_doublet_strength, 0.0, 0.0),
    //         b: Vector3::new(0.0, 0.0, -1.0),
    //     },
    // };
    // let mirror_potential_doublet_strength = -2.0 * h.powi(2) * stokeslet_strength;
    // let potential_doublet_mirror = Singularity {
    //     point: stokeslet_mirror_point,
    //     params: SingularityParams::PotentialDoublet {
    //         d: Vector3::new(mirror_potential_doublet_strength, 0.0, 0.0),
    //     },
    // };

    let mut singularities: Vec<(flow::VectorLabel, Singularity)> =
        vec![(flow::VectorLabel(String::from("Origin")), stokeslet_origin)];

    singularities.extend(
        stokeslet_mirror(stokeslet_origin_point, d, stokeslet_force)
            .into_iter()
            .map(|s| (flow::VectorLabel(String::from("Mirror at +z")), s)),
    );
    singularities.extend(
        stokeslet_mirror(stokeslet_origin_point, -d, stokeslet_force)
            .into_iter()
            .map(|s| (flow::VectorLabel(String::from("Mirror at -z")), s)),
    );
    // (
    //     flow::VectorLabel(String::from("Mirror: Stokeslet")),
    //     stokeslet_mirror,
    // ),
    // (
    //     flow::VectorLabel(String::from("Mirror: Stokes Doublet")),
    //     stokes_doublet_mirror,
    // ),
    // (
    //     flow::VectorLabel(String::from("Mirror: Potential Doublet")),
    //     potential_doublet_mirror,
    // ),
    // (
    //     flow::VectorLabel(String::from("Stresslet, Origin")),
    //     Singularity {
    //         point: Point3::origin(),
    //         params: SingularityParams::Stresslet {
    //             a: Vector3::new(1.0, 0.0, 0.0),
    //             b: Vector3::new(0.0, 0.0, -1.0),
    //         },
    //     },
    // ),
    // (
    //     flow::VectorLabel(String::from("StokesDoublet, Origin")),
    //     Singularity {
    //         point: Point3::origin(),
    //         params: SingularityParams::StokesDoublet {
    //             a: Vector3::new(1.0, 0.0, 0.0),
    //             b: Vector3::new(0.0, 0.0, -1.0),
    //         },
    //     },
    // ),
    // (
    //     flow::VectorLabel(String::from("Rotlet, Origin")),
    //     Singularity {
    //         point: Point3::origin(),
    //         params: SingularityParams::Rotlet {
    //             c: Vector3::new(0.0, 1.0, 0.0),
    //         },
    //     },
    // ),
    // (
    //     flow::VectorLabel(String::from("PotentialDoublet, Origin")),
    //     Singularity {
    //         point: Point3::origin(),
    //         params: SingularityParams::PotentialDoublet {
    //             d: Vector3::new(0.0, 1.0, 0.0),
    //         },
    //     },
    // ),
    // ];

    let markers: Vec<Marker> = sample_rs
        .iter()
        .map(|r| Marker {
            r: *r,
            vs: flow::VectorSet(
                singularities
                    .iter()
                    .map(|(label, s)| (label.clone(), s.eval(*r).0))
                    .collect(),
            ),
        })
        .collect();

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(env)
        .insert_resource(MarkerSet(markers))
        .insert_resource(SingularitySet(singularities))
        .insert_resource(FlowViewState::new(0.05))
        .add_systems(
            Startup,
            (
                add_flow_markers,
                add_singularities,
                add_camera,
                add_axis_arrows,
            ),
        )
        .add_systems(PostStartup, add_flow)
        .add_systems(Startup, environment::add_boundaries)
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

    info!("Done!");
}
