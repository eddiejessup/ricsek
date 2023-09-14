use std::{f32::consts::FRAC_PI_2, time::Duration};

use bevy::{
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    pbr::wireframe::WireframePlugin,
    prelude::*,
    render::{render_resource::WgpuFeatures, settings::WgpuSettings, RenderPlugin},
    time::common_conditions::on_timer,
};
use nalgebra::Point3;
use ricsek::view::{
    common::{add_axis_arrows, nalgebra_to_glam_vec},
    environment::Environment,
    flow::FlowViewState,
    pan_orbit_camera::{add_camera, pan_orbit_camera_update},
    *,
};
use structopt::StructOpt;

#[derive(Component)]
pub enum AgentBodyComponent {
    Back,
    Front,
    Rod,
}

pub fn add_agents(
    mut commands: Commands,
    sim_states: Res<SimStates>,
    env: Res<environment::Environment>,
    config: Res<SetupRes>,
    view_state: Res<ViewState>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let view_i = view_state.i;
    let cur_sim_state = &sim_states.0[view_i];

    let config = &config.0;

    let radius = config.parameters.agent_radius;

    let back_material = materials.add(StandardMaterial::from(Color::GREEN.with_a(0.9)));
    let front_material = materials.add(StandardMaterial::from(Color::RED.with_a(0.9)));
    let rod_material = materials.add(StandardMaterial::from(Color::BLACK.with_a(0.9)));

    let sphere_mesh = meshes.add(
        shape::UVSphere {
            radius: env.transform_coord(radius),
            sectors: 18,
            stacks: 9,
        }
        .into(),
    );
    let rod_mesh = meshes.add(
        shape::Cylinder {
            radius: env.transform_coord(0.1e-6),
            height: env.transform_coord(config.parameters.agent_inter_sphere_length),
            resolution: 18,
            segments: 1,
        }
        .into(),
    );

    cur_sim_state.agents.iter().enumerate().for_each(|(i, _a)| {
        commands
            // Represents the point, don't adjust its orientation,
            // just translate it to the agent's centre-of-mass position.
            .spawn((
                SpatialBundle::default(),
                AgentId(i),
                flow::VectorSet(vec![]),
            ))
            .with_children(|parent| {
                parent.spawn((
                    PbrBundle {
                        mesh: sphere_mesh.clone(),
                        material: back_material.clone(),
                        ..default()
                    },
                    AgentId(i),
                    AgentBodyComponent::Back,
                ));
                parent.spawn((
                    PbrBundle {
                        mesh: sphere_mesh.clone(),
                        material: front_material.clone(),
                        ..default()
                    },
                    AgentId(i),
                    AgentBodyComponent::Front,
                ));
                parent
                    .spawn((
                        SpatialBundle::default(),
                        AgentId(i),
                        AgentBodyComponent::Rod,
                    ))
                    .with_children(|rod_parent| {
                        rod_parent.spawn((PbrBundle {
                            mesh: rod_mesh.clone(),
                            material: rod_material.clone(),
                            transform: Transform::from_rotation(Quat::from_rotation_x(-FRAC_PI_2)),
                            ..default()
                        },));
                    });
            });
    });
}

fn update_agent_points(
    sim_states: Res<SimStates>,
    env: Res<Environment>,
    view_state: Res<ViewState>,
    mut q_point: Query<(&AgentId, &mut flow::VectorSet, &mut Transform)>,
) {
    let view_i = view_state.i;
    let cur_sim_state = &sim_states.0[view_i];
    let opt_summary = &cur_sim_state.summary;
    match opt_summary {
        None => debug!("No agent summary for view_i: {}", view_i),
        Some(_) => debug!("Agent summary for view_i: {}", view_i),
    };

    debug!("Updating positions for agents, to view_i: {}", view_i);
    for (agent_id, mut vector_set, mut transform) in &mut q_point {
        let agent = &cur_sim_state.agents[agent_id.0];
        *transform = Transform::from_translation(env.transformed_vec3(agent.seg.centroid()));

        if let Some(agent_summaries) = opt_summary {
            let agent_summary = &agent_summaries[agent_id.0];
            *vector_set = flow::VectorSet(vec![
                (
                    flow::VectorLabel("f_agent_electro".to_string()),
                    agent_summary.f_agent_electro,
                ),
                (
                    flow::VectorLabel("torque_agent_electro".to_string()),
                    1e6 * agent_summary.torque_agent_electro,
                ),
                (
                    flow::VectorLabel("f_agent_hydro".to_string()),
                    agent_summary.f_agent_hydro,
                ),
                (
                    flow::VectorLabel("f_propulsion".to_string()),
                    agent_summary.f_propulsion,
                ),
                (
                    flow::VectorLabel("f_boundary_electro".to_string()),
                    agent_summary.f_boundary_electro,
                ),
                (
                    flow::VectorLabel("torque_boundary_electro".to_string()),
                    1e6 * agent_summary.torque_boundary_electro,
                ),
                (
                    flow::VectorLabel("f_singularity".to_string()),
                    agent_summary.f_singularity,
                ),
            ])
        }
    }
}

fn update_agent_orientations(
    env: Res<Environment>,
    sim_states: Res<SimStates>,
    view_state: Res<ViewState>,
    mut q_ag: Query<(&AgentId, &mut Transform, &AgentBodyComponent)>,
) {
    let view_i = view_state.i;
    let cur_sim_state = &sim_states.0[view_i];
    debug!("Updating orientations for agents, to view_i: {}", view_i);
    for (agent_id, mut transform, agent_body) in &mut q_ag {
        let agent = &cur_sim_state.agents[agent_id.0];
        *transform = match agent_body {
            AgentBodyComponent::Front => Transform::from_translation(
                env.transformed_vec3(Point3::origin() + agent.seg.start_end() * 0.5),
            )
            .looking_to(nalgebra_to_glam_vec(&agent.seg.start_end()), Vec3::Z),
            AgentBodyComponent::Back => Transform::from_translation(
                env.transformed_vec3(Point3::origin() - agent.seg.start_end() * 0.5),
            )
            .looking_to(nalgebra_to_glam_vec(&agent.seg.start_end()), Vec3::Z),
            AgentBodyComponent::Rod => Transform::IDENTITY
                .looking_to(nalgebra_to_glam_vec(&agent.seg.start_end()), Vec3::Z),
        }
    }
}

fn change_view(
    keyboard_input: Res<Input<KeyCode>>,
    sim_states: Res<SimStates>,
    mut view_state: ResMut<ViewState>,
) {
    if keyboard_input.just_pressed(KeyCode::Key0) {
        view_state.i = 0;
        return;
    }

    let backward = if keyboard_input.pressed(KeyCode::Left) {
        Some(true)
    } else if keyboard_input.pressed(KeyCode::Right) {
        Some(false)
    } else {
        None
    };
    if let Some(backward) = backward {
        view_state.i = common::increment_step(
            view_state.i,
            backward,
            sim_states.0.len() - 1,
            view_state.stepsize,
        );
        return;
    }

    let slowards = if keyboard_input.pressed(KeyCode::Down) {
        Some(true)
    } else if keyboard_input.pressed(KeyCode::Up) {
        Some(false)
    } else {
        None
    };
    if let Some(slowards) = slowards {
        let new_step = view_state.stepsize as i64 + (if slowards { -1 } else { 1 });
        view_state.stepsize = if new_step >= 1 { new_step as usize } else { 1 };
        return;
    }
}

#[derive(Debug, StructOpt)]
#[structopt(name = "ricsek_view", about = "View a run results...")]
pub struct ViewCli {
    #[structopt(short = "w", long = "window-size", default_value = "800.0")]
    pub window_size: f64,

    #[structopt(short = "r", long = "run_id")]
    pub run_id: Option<usize>,
}

fn main() {
    env_logger::init();
    let args = ViewCli::from_args();

    let conn = &mut ricsek::db::establish_connection();

    let run_id = match args.run_id {
        Some(run_id) => run_id,
        None => {
            warn!("No run_id specified, using latest run_id");
            ricsek::db::read_latest_run_id(conn)
        }
    };

    let setup = ricsek::db::read_run(conn, run_id);
    let sim_states = ricsek::db::read_run_sim_states(conn, run_id);

    let env = Environment {
        boundaries: setup.parameters.boundaries.clone(),
        arrow_length: 2.0e-6,
        length_factor: 1e6,
    };

    info!("Got {} sim-states", sim_states.len());

    App::new()
        .add_plugins((
            DefaultPlugins.set(RenderPlugin {
                wgpu_settings: WgpuSettings {
                    features: WgpuFeatures::POLYGON_MODE_LINE,
                    ..default()
                },
            }),
            WireframePlugin,
        ))
        .add_plugins((
            LogDiagnosticsPlugin::default(),
            FrameTimeDiagnosticsPlugin::default(),
        ))
        .insert_resource(env)
        .insert_resource(SimStates(sim_states))
        .insert_resource(SetupRes(setup))
        .insert_resource(FlowViewState::new(10000))
        .insert_resource(ViewState::default())
        .add_systems(Startup, add_camera)
        // .add_systems(Startup, add_axis_arrows)
        .add_systems(Startup, add_agents)
        .add_systems(Startup, environment::add_environment)
        .add_systems(PostStartup, flow::add_flow)
        .add_systems(Update, pan_orbit_camera_update)
        .add_systems(
            Update,
            (
                update_agent_points,
                update_agent_orientations,
                change_view.run_if(on_timer(Duration::from_secs_f64(TIME_STEP))),
            ),
        )
        .add_systems(
            PostUpdate,
            (
                flow::update_flow,
                flow::change_view.run_if(on_timer(Duration::from_secs_f64(TIME_STEP))),
            ),
        )
        .add_systems(Update, bevy::window::close_on_esc)
        .run();

    info!("Done!");
}
