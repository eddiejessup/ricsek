use std::time::Duration;

use bevy::{
    pbr::PbrBundle, prelude::*, render::view::visibility::Visibility,
    time::common_conditions::on_timer,
};
use nalgebra::{Point3, Vector3};
use ricsek::{
    config::setup::parameters::simulation::{AxisBoundaryConfig, BoundaryConfig},
    dynamics::stokes_solutions::*,
    view::*,
};

#[derive(Resource)]
struct Samples(Vec<Point3<f64>>);

#[derive(Debug, Component, Clone)]
enum SingularityParams {
    Stokeslet { a: Vector3<f64> },
    StokesDoublet { a: Vector3<f64>, b: Vector3<f64> },
    Rotlet { c: Vector3<f64> },
    Stresslet { a: Vector3<f64>, b: Vector3<f64> },
    PotentialDoublet { d: Vector3<f64> },
}

impl SingularityParams {
    fn eval(&self, r: Vector3<f64>) -> Vector3<f64> {
        match self {
            SingularityParams::Stokeslet { a } => stokeslet_u(*a, r),
            SingularityParams::StokesDoublet { a, b } => stokes_doublet_chwang_u(*a, *b, r),
            SingularityParams::Rotlet { c } => rotlet_chwang_u(*c, r),
            SingularityParams::Stresslet { a, b } => stresslet_chwang_u(*a, *b, r),
            SingularityParams::PotentialDoublet { d } => potential_doublet_chwang_u(*d, r),
        }
    }
}

#[derive(Clone)]
struct Singularity {
    point: Point3<f64>,
    params: SingularityParams,
}

#[derive(Resource)]
struct Singularities(Vec<Singularity>);

fn add_samples(
    mut commands: Commands,
    samples: Res<Samples>,
    env: Res<EnvironmentRes>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let red: Handle<StandardMaterial> = materials.add(StandardMaterial::from(Color::RED));
    let cube: Handle<Mesh> = meshes.add((shape::Cube { size: 1.0 }).into()).into();
    for (i, r) in samples.0.iter().enumerate() {
        commands.spawn((
            PbrBundle {
                mesh: cube.clone(),
                material: red.clone(),
                transform: Transform::from_translation(env.transformed_vec3(*r)),
                ..default()
            },
            SampleId(i),
        ));
    }
}

fn add_flow(
    mut commands: Commands,
    samples: Res<Samples>,
    singularities: Res<Singularities>,
    env: Res<EnvironmentRes>,
    view_state: Res<ViewState>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let selected_singularity = &singularities.0[view_state.i];

    commands.spawn((
        selected_singularity.params.clone(),
        PbrBundle {
            mesh: meshes.add(
                shape::UVSphere {
                    radius: 1.0,
                    ..default()
                }
                .into(),
            ),
            material: materials.add(StandardMaterial::from(Color::PURPLE)),
            transform: Transform::from_translation(
                env.transformed_vec3(selected_singularity.point),
            ),
            ..default()
        },
    ));

    // Sample flow vectors.
    let arrow: Handle<Mesh> = meshes.add(
        bevy_more_shapes::Cone {
            radius: 0.3,
            height: 0.4,
            segments: 32,
        }
        .into(),
    );
    let cylinder: Handle<Mesh> = meshes.add(
        shape::Cylinder {
            radius: 0.1,
            height: 1.0,
            resolution: 32,
            ..default()
        }
        .into(),
    );
    for (i, _) in samples.0.iter().enumerate() {
        let child_material: Handle<StandardMaterial> =
            materials.add(StandardMaterial::from(Color::PURPLE));
        commands
            .spawn((SampleId(i), SpatialBundle::default()))
            .with_children(|parent| {
                parent.spawn((
                    PbrBundle {
                        mesh: arrow.clone(),
                        material: child_material.clone(),
                        transform: Transform::from_translation(Vec3::Y * 0.5),
                        ..default()
                    },
                    FlowVectorId,
                ));
                parent.spawn((
                    PbrBundle {
                        mesh: cylinder.clone(),
                        material: child_material.clone(),
                        ..default()
                    },
                    FlowVectorId,
                ));
            });
    }
}

fn update_flow(
    samples: Res<Samples>,
    env: Res<EnvironmentRes>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut set: ParamSet<(
        Query<(&Transform, &SingularityParams)>,
        Query<(&mut Transform, &mut Visibility, &SampleId, &Children)>,
    )>,
    q_flow: Query<(&Handle<StandardMaterial>, &FlowVectorId)>,
) {
    let q_singularity = set.p0();

    let (sing_transform, singularity_params) = q_singularity.single();

    let sing_r = env.inverted_vec3(sing_transform.translation);

    let vs: Vec<Vector3<f64>> = samples
        .0
        .iter()
        .map(|r| singularity_params.eval(sing_r - *r))
        .collect();

    // Get maximum velocity magnitude.
    let (min_vel_log, max_vel_log) = vs.iter().map(|v| v.magnitude().log10()).fold(
        (f64::INFINITY, f64::NEG_INFINITY),
        |(mn, mx): (f64, f64), b| {
            (
                if b > f64::NEG_INFINITY { mn.min(b) } else { mn },
                mx.max(b),
            )
        },
    );

    let g = colorgrad::viridis();
    println!("Min velocity: {}", min_vel_log);
    println!("Max velocity: {}", max_vel_log);

    let mut q_samples = set.p1();

    // Iterate over (p, v) pairs to draw them.
    for (mut transform, mut visibility, sample_id, children) in q_samples.iter_mut() {
        let v = vs[sample_id.0];
        let r = samples.0[sample_id.0];

        // Get a magnitude in [0.0, 1.0] normalized on the max velocity, on a log scale.
        let mag_scale = (v.magnitude().log10() - min_vel_log) / (max_vel_log - min_vel_log);

        *visibility = if mag_scale < 0.5 {
            Visibility::Hidden
        } else {
            Visibility::Visible
        };

        // Map normalized velocity to a point on the color spectrum.
        let mag_color = g.at(mag_scale);

        // If i in in a fixed set of values
        if [0, 5, 20].contains(&sample_id.0) {
            println!("Sample {}: r: {:?}, v: {:?}", sample_id.0, r, v);
            println!("v: {:?}", v);
            println!("mag_scale: {}", mag_scale);
            println!("color: {:?}", mag_color);
            println!("");
        }

        for &child in children.iter() {
            let (color_handle, _flow_vector) = q_flow.get(child).unwrap();
            let color_mat = materials.get_mut(color_handle).unwrap();
            color_mat.base_color = Color::rgba(
                mag_color.r as f32,
                mag_color.g as f32,
                mag_color.b as f32,
                mag_color.a as f32,
            );
        }

        match nalgebra_to_glam_vec(&v).try_normalize() {
            Some(glam_u) => {
                *transform = Transform::IDENTITY
                    .with_translation(env.transformed_vec3(r))
                    .with_scale(Vec3::splat(env.transform_coord(env.arrow_length)))
                    .with_rotation(Quat::from_rotation_arc(Vec3::Y, glam_u))
            }
            None => {
                *visibility = Visibility::Hidden;
            }
        };
    }
    println!("");
    println!("");
}

fn change_view(
    keyboard_input: Res<Input<KeyCode>>,
    env: Res<EnvironmentRes>,
    mut view_state: ResMut<ViewState>,
    singularities: Res<Singularities>,
    mut q_singularity: Query<(&mut Transform, &mut SingularityParams)>,
) {
    let (mut singularity_transform, mut singularity_params) = q_singularity.single_mut();

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
            let new_sing = singularities.0[view_state.i].clone();
            *singularity_params = new_sing.params;
            *singularity_transform =
                Transform::from_translation(env.transformed_vec3(new_sing.point));
        }
    }

    let mag_down = if keyboard_input.pressed(KeyCode::Down) {
        Some(true)
    } else if keyboard_input.pressed(KeyCode::Up) {
        Some(false)
    } else {
        None
    };

    if let Some(mag_down) = mag_down {
        adjust_singularity_params(&mut singularity_params, if mag_down { 1.1 } else { 0.9 });
        println!("Singularity params: {:?}", singularity_params);
    }
}

fn adjust_singularity_params(p: &mut SingularityParams, factor: f64) {
    match p {
        SingularityParams::Stokeslet { a } => {
            *a *= factor;
        }
        SingularityParams::StokesDoublet { a, .. } => {
            *a *= factor;
        }
        SingularityParams::Rotlet { c } => {
            *c *= factor;
        }
        SingularityParams::Stresslet { a, .. } => {
            *a *= factor;
        }
        SingularityParams::PotentialDoublet { d } => {
            *d *= factor;
        }
    }
}

#[derive(Resource)]
pub struct ViewState {
    pub i: usize,
}

impl ViewState {
    pub fn new() -> Self {
        Self { i: 0 }
    }
}

fn main() {
    let env = EnvironmentRes {
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
                l: 20.0e-6,
                closed: true,
            },
        )),
        arrow_length: 10.0e-6,
    };

    let samples: Vec<Point3<f64>> = ricsek::math::grid(env.boundaries.l(), 1000);

    let singularities = vec![
        Singularity {
            point: Point3::origin(),
            params: SingularityParams::Stokeslet {
                a: Vector3::new(1.0, 1.0, 1.0),
            },
        },
        Singularity {
            point: Point3::origin(),
            params: SingularityParams::Stresslet {
                a: Vector3::new(1.0, 0.0, 0.0),
                b: Vector3::new(1.0, 0.0, 0.0),
            },
        },
        Singularity {
            point: Point3::origin(),
            params: SingularityParams::StokesDoublet {
                a: Vector3::new(1.0, 0.0, 0.0),
                b: Vector3::new(0.0, -1.0, 0.0),
            },
        },
        Singularity {
            point: Point3::origin(),
            params: SingularityParams::Rotlet {
                c: Vector3::new(0.0, 1.0, 0.0),
            },
        },
        Singularity {
            point: Point3::origin(),
            params: SingularityParams::PotentialDoublet {
                d: Vector3::new(0.0, 1.0, 0.0),
            },
        },
    ];

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(env)
        .insert_resource(Samples(samples))
        .insert_resource(Singularities(singularities))
        .insert_resource(ViewState::new())
        .add_systems(Startup, (add_samples, add_flow, add_camera))
        .add_systems(Startup, add_environment)
        .add_systems(Update, pan_orbit_camera)
        .add_systems(
            Update,
            (
                change_view.run_if(on_timer(Duration::from_secs_f64(TIME_STEP))),
                update_flow,
                bevy::window::close_on_esc,
                //         cursor_system,
            ),
        )
        .run();

    println!("Done!");
}
