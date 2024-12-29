use std::time::Duration;

use bevy::{color::palettes::css, prelude::*, time::common_conditions::on_timer};
use nalgebra::{zero, Point3, Vector3};
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
pub struct SingularitySet(pub Vec<(String, Singularity)>);

// Helper to add markers just there to hold a vector-set.
pub fn add_flow_markers(
    mut commands: Commands,
    markers: Res<MarkerSet>,
    // mut meshes: ResMut<Assets<Mesh>>,
    // mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Marker bases.
    // let red: Handle<StandardMaterial> = materials.add(StandardMaterial::from(Color::RED));
    // let cube: Handle<Mesh> = meshes.add((shape::Cube { size: 1.0 }).into()).into();

    for sample in markers.0.iter() {
        commands.spawn((
            // Mesh3d(cube.clone()),
            // MeshMaterial3d(red.clone()),
            Transform::from_translation(point3_to_gvec3(&sample.r)),
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
        materials.add(StandardMaterial::from(Color::from(css::YELLOW)));
    let mesh_handle: Handle<Mesh> = meshes.add(Cuboid::from_length(1.0).mesh());

    for (label, sing) in singularities.0.iter() {
        commands.spawn((
            Mesh3d(mesh_handle.clone()),
            MeshMaterial3d(material_handle.clone()),
            Transform::from_translation(point3_to_gvec3(&sing.point)),
            label.clone(),
        ));
    }
}

fn stokeslet_image(
    p: Point3<f64>,
    d: Vector3<f64>,
    stokeslet_force: Vector3<f64>,
) -> Vec<(String, Singularity)> {
    let d_direction = d.normalize();
    let h = d.norm();

    let stokeslet_force_normal = (stokeslet_force.dot(&d) / h.powi(2)) * d;
    let stokeslet_force_tangent = stokeslet_force - stokeslet_force_normal;

    let stokeslet_strength_normal = stokeslet_force_normal.norm();
    let stokeslet_strength_tangent = stokeslet_force_tangent.norm();

    let stokeslet_direction_normal = if stokeslet_strength_normal > 0.0 {
        stokeslet_force_normal.normalize()
    } else {
        zero()
    };
    let stokeslet_direction_tangent = if stokeslet_strength_tangent > 0.0 {
        stokeslet_force_tangent.normalize()
    } else {
        zero()
    };

    let stokes_doublet_normal_mag = (2.0 * h * stokeslet_strength_normal).sqrt();
    let stokes_doublet_tangent_mag = (2.0 * h * stokeslet_strength_tangent).sqrt();
    let potential_doublet_mag = 2.0 * h.powi(2);

    [
        (
            SingularityParams::Stokeslet {
                a: -stokeslet_force,
            },
            "Mirror stokeslet",
        ),
        (
            SingularityParams::StokesDoublet {
                a: -stokes_doublet_normal_mag * stokeslet_direction_normal,
                b: -stokes_doublet_normal_mag * d_direction,
            },
            "Mirror stokes doublet, normal",
        ),
        (
            SingularityParams::PotentialDoublet {
                d: potential_doublet_mag * stokeslet_force_normal,
            },
            "Mirror potential doublet, normal",
        ),
        (
            SingularityParams::StokesDoublet {
                a: -stokes_doublet_tangent_mag * stokeslet_direction_tangent,
                b: stokes_doublet_tangent_mag * d_direction,
            },
            "Mirror stokes doublet, tangential",
        ),
        (
            SingularityParams::PotentialDoublet {
                d: -potential_doublet_mag * stokeslet_force_tangent,
            },
            "Mirror potential doublet, tangential",
        ),
    ]
    .iter()
    .map(|(params, label)| {
        (
            String::from(*label),
            Singularity {
                point: p + 2.0 * d,
                params: params.clone(),
            },
        )
    })
    .collect()
}

fn main() {
    env_logger::init();
    let bc = BoundaryConfig(Vector3::new(
        AxisBoundaryConfig {
            l: 100.0,
            closed: true,
        },
        AxisBoundaryConfig {
            l: 100.0,
            closed: true,
        },
        AxisBoundaryConfig {
            l: 100.0,
            closed: true,
        },
    ));
    let env = environment::Environment {
        boundaries: Some(bc.clone()),
    };

    let mut sample_rs = Vec::new();
    // for p in ricsek::geometry::grid_3d(1.2 * env.boundaries.clone().unwrap().l(), 4000) {
    //     sample_rs.push(p);
    // }

    let sample_l = nalgebra::Vector2::new(3.0 * bc.l().x, 3.0 * bc.l().y);
    let sample_z = bc.l_half().z;
    let step = 5.0;
    for p in ricsek::geometry::grid_2d(sample_l, step) {
        sample_rs.push(Point3::new(p.x, p.y, sample_z));
        // sample_rs.push(Point3::new(p.x, p.y, -sample_z));
    }

    let stokeslet_strength = 100.0;
    // let stokeslet_force = Vector3::new(0.0, 0.0, stokeslet_strength);
    // let stokeslet_force = Vector3::new(0.0, 0.0, -stokeslet_strength);
    // let stokeslet_force = Vector3::new(stokeslet_strength, 0.0, 0.0);
    // let stokeslet_force = Vector3::new(-stokeslet_strength, 0.0, 0.0);
    // let stokeslet_force = Vector3::new(0.0, stokeslet_strength, 0.0);
    // let stokeslet_force = Vector3::new(0.0, -stokeslet_strength, 0.0);
    // let stokeslet_force = Vector3::new(1.0, 1.0, 1.0).normalize() * stokeslet_strength;
    let stokeslet_force = Vector3::new(1.0, 1.0, 0.0).normalize() * stokeslet_strength;

    let stokeslet_origin_point = Point3::new(0.0, 0.0, 2.0);
    let stokeslet_origin = Singularity {
        point: stokeslet_origin_point,
        params: SingularityParams::Stokeslet { a: stokeslet_force },
    };
    let d = Vector3::new(0.0, 0.0, bc.0.z.l_half() - stokeslet_origin_point.z);

    let mut singularities: Vec<(String, Singularity)> =
        vec![("Origin".to_string(), stokeslet_origin)];

    singularities.extend(stokeslet_image(stokeslet_origin_point, d, stokeslet_force));

    let markers: Vec<Marker> = sample_rs
        .iter()
        .map(|r| Marker {
            r: *r,
            vs: flow::VectorSet(
                singularities
                    .iter()
                    .map(|(label, s)| {
                        let v = s.eval(*r).0;
                        println!("{}: {}", label.0, v);
                        (label.clone(), v)
                    })
                    .collect(),
            ),
        })
        .collect();

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(env)
        .insert_resource(MarkerSet(markers))
        .insert_resource(SingularitySet(singularities))
        .insert_resource(FlowViewState::default())
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
        // .add_systems(Startup, environment::add_boundaries)
        .add_systems(Update, pan_orbit_camera_update)
        .add_systems(
            Update,
            (
                flow::update_flow_markers.run_if(on_timer(Duration::from_secs_f64(TIME_STEP))),
                update_flow,
                common::close_on_esc,
            ),
        )
        .run();

    info!("Done!");
}
