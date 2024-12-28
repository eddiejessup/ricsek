use std::time::Duration;

use bevy::{
    color::palettes::css,
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    prelude::*,
    render::mesh::SphereKind,
    time::common_conditions::on_timer,
};
use clap::Parser;
use nalgebra::zero;
use ricsek::{
    config::{run::RunContext, setup::SetupConfig},
    dynamics::{run_steps, StepSummary},
    state::SimState,
    view::{
        common::{add_axis_arrows, point3_to_gvec3, vec3_to_gvec3},
        environment::Environment,
        flow::FlowViewState,
        pan_orbit_camera::{add_camera, pan_orbit_camera_update},
        *,
    },
};

const F_COEFF: f64 = 1e6;
const TORQUE_COEFF: f64 = 1e6;
const V_COEFF: f64 = 1e-1;

// Resources.

#[derive(Resource)]
struct SimViewState {
    pub i: usize,
    pub checkpoint_stepsize: usize,
    pub sim_stepsize: usize,
}

impl Default for SimViewState {
    fn default() -> Self {
        Self {
            i: 0,
            checkpoint_stepsize: 1,
            sim_stepsize: 10,
        }
    }
}

#[derive(Resource)]
struct SimStates(pub Vec<(SimState, Option<StepSummary>)>);

#[derive(Resource)]
struct SetupConfigRes(pub SetupConfig);

/// Helper resource for tracking our asset
// #[derive(Resource)]
// struct GltfAsset(Handle<Gltf>);

// / Resources.

// Components.

#[derive(Component)]
struct AgentId(pub usize);

#[derive(Component)]
enum AgentBodyComponent {
    // Back,
    // Front,
    Rod,
}

#[derive(Component)]
struct FlowMarkerId(pub usize);

// / Components.

fn add_flow_markers(
    mut commands: Commands,
    setup_config: Res<SetupConfigRes>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Marker bases.
    let red: Handle<StandardMaterial> =
        materials.add(StandardMaterial::from(Color::from(css::RED)));
    let cube: Handle<Mesh> = meshes.add(Cuboid::from_length(0.2).mesh().build());

    for (i, sample) in setup_config.0.sample_points.iter().enumerate() {
        commands.spawn((
            Mesh3d(cube.clone()),
            MeshMaterial3d(red.clone()),
            Transform::from_translation(point3_to_gvec3(sample)),
            FlowMarkerId(i),
            flow::VectorSet(vec![]),
        ));
    }
}

fn update_flow_markers(
    sim_states: Res<SimStates>,
    view_state: Res<SimViewState>,
    mut q_point: Query<(&FlowMarkerId, &mut flow::VectorSet)>,
) {
    let view_i = view_state.i;
    let (_cur_sim_state, cur_step_summary) = &sim_states.0[view_i];

    debug!("Updating flow for agents, to view_i: {}", view_i);
    if let Some(StepSummary {
        agent_summaries: _,
        fluid_flow,
    }) = cur_step_summary
    {
        for (marker_id, mut vector_set) in &mut q_point {
            let marker_fluid_v = fluid_flow[marker_id.0];
            *vector_set = flow::VectorSet(vec![
                (flow::VectorLabel("f_agent_electro".to_string()), zero()),
                (
                    flow::VectorLabel("torque_agent_electro".to_string()),
                    zero(),
                ),
                (flow::VectorLabel("f_propulsion".to_string()), zero()),
                (flow::VectorLabel("f_boundary_electro".to_string()), zero()),
                (
                    flow::VectorLabel("torque_boundary_electro".to_string()),
                    zero(),
                ),
                (
                    flow::VectorLabel("v_fluid".to_string()),
                    V_COEFF * marker_fluid_v,
                ),
            ])
        }
    }
}

fn add_agents(
    mut commands: Commands,
    sim_states: Res<SimStates>,
    setup_config: Res<SetupConfigRes>,
    view_state: Res<SimViewState>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    _asset_server: Res<AssetServer>,
) {
    let view_i = view_state.i;
    let (cur_sim_state, _) = &sim_states.0[view_i];

    let params = &setup_config.0.parameters;

    let radius = params.agent_radius;

    let _back_material = materials.add(StandardMaterial::from(
        Color::from(css::GREEN).with_alpha(0.9),
    ));
    let _front_material = materials.add(StandardMaterial::from(
        Color::from(css::RED).with_alpha(0.9),
    ));
    let rod_material = materials.add(StandardMaterial::from(Color::BLACK.with_alpha(0.9)));

    let _sphere_mesh = meshes.add(
        Sphere::new(radius as f32)
            .mesh()
            .kind(SphereKind::Uv {
                sectors: 18,
                stacks: 9,
            })
            .build(),
    );
    let rod_mesh = meshes.add(
        Cylinder::new(0.1, params.agent_inter_sphere_length as f32)
            .mesh()
            .segments(6)
            .resolution(18)
            .build(),
    );

    // let cell_scene_handle = asset_server.load("models/cell.glb#Scene0");

    cur_sim_state.agents.iter().enumerate().for_each(|(i, _a)| {
        commands
            // Represents the point, don't adjust its orientation,
            // just translate it to the agent's centre-of-mass position.
            .spawn((
                Transform::default(),
                Visibility::default(),
                AgentId(i),
                flow::VectorSet(vec![]),
            ))
            .with_children(|parent| {
                parent.spawn((
                    // Add cylinder
                    Mesh3d::from(rod_mesh.clone()),
                    MeshMaterial3d(rod_material.clone()),
                    AgentId(i),
                    AgentBodyComponent::Rod,
                ));
            });
    });
}

fn update_agent_points(
    sim_states: Res<SimStates>,
    view_state: Res<SimViewState>,
    mut q_point: Query<(&AgentId, &mut flow::VectorSet, &mut Transform)>,
) {
    let view_i = view_state.i;
    let (cur_sim_state, cur_step_summary) = &sim_states.0[view_i];

    debug!("Updating positions for agents, to view_i: {}", view_i);
    for (agent_id, mut vector_set, mut transform) in &mut q_point {
        let agent = &cur_sim_state.agents[agent_id.0];
        *transform = Transform::from_translation(point3_to_gvec3(&agent.seg.centroid()));

        if let Some(StepSummary {
            agent_summaries,
            fluid_flow: _,
        }) = cur_step_summary
        {
            let agent_summary = &agent_summaries[agent_id.0];
            *vector_set = flow::VectorSet(vec![
                (
                    flow::VectorLabel("f_agent_electro".to_string()),
                    F_COEFF * agent_summary.f_agent_electro,
                ),
                (
                    flow::VectorLabel("torque_agent_electro".to_string()),
                    TORQUE_COEFF * agent_summary.torque_agent_electro,
                ),
                (
                    flow::VectorLabel("f_propulsion".to_string()),
                    F_COEFF * agent_summary.f_propulsion,
                ),
                (
                    flow::VectorLabel("f_boundary_electro".to_string()),
                    F_COEFF * agent_summary.f_boundary_electro,
                ),
                (
                    flow::VectorLabel("torque_boundary_electro".to_string()),
                    TORQUE_COEFF * agent_summary.torque_boundary_electro,
                ),
                (
                    flow::VectorLabel("v_fluid".to_string()),
                    V_COEFF * agent_summary.v_fluid,
                ),
            ])
        }
    }
}

fn update_agent_orientations(
    sim_states: Res<SimStates>,
    view_state: Res<SimViewState>,
    mut q_ag: Query<(&AgentId, &mut Transform, &AgentBodyComponent)>,
) {
    let view_i = view_state.i;
    let (cur_sim_state, _) = &sim_states.0[view_i];
    debug!("Updating orientations for agents, to view_i: {}", view_i);
    for (agent_id, mut transform, agent_body) in &mut q_ag {
        let agent = &cur_sim_state.agents[agent_id.0];
        *transform = match agent_body {
            // AgentBodyComponent::Front => Transform::from_translation(point3_to_gvec3(
            //     &(Point3::origin() + agent.seg.start_end() * 0.5),
            // ))
            // .looking_to(vec3_to_gvec3(&agent.seg.start_end()), Vec3::Z),
            // AgentBodyComponent::Back => Transform::from_translation(point3_to_gvec3(
            //     &(Point3::origin() - agent.seg.start_end() * 0.5),
            // ))
            // .looking_to(vec3_to_gvec3(&agent.seg.start_end()), Vec3::Z),
            AgentBodyComponent::Rod => {
                Transform::IDENTITY.looking_to(vec3_to_gvec3(&agent.seg.start_end()), Vec3::Z)
            }
        }
    }
}

fn update_timestep(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    setup_config: Res<SetupConfigRes>,
    mut sim_states: ResMut<SimStates>,
    mut view_state: ResMut<SimViewState>,
) {
    if keyboard_input.just_pressed(KeyCode::Digit0) {
        view_state.i = 0;
        return;
    }

    let backward = if keyboard_input.pressed(KeyCode::ArrowLeft) {
        Some(true)
    } else if keyboard_input.pressed(KeyCode::ArrowRight) {
        Some(false)
    } else {
        None
    };

    if let Some(backward) = backward {
        if !backward && view_state.i == sim_states.0.len() - 1 {
            let mut final_state = (sim_states.0[view_state.i]).0.clone();

            let run_context = RunContext::new(
                &setup_config.0.parameters,
                final_state.agents.len(),
                setup_config.0.sample_points.clone(),
            );

            let summary = run_steps(
                &setup_config.0.parameters,
                &mut final_state,
                &run_context,
                view_state.sim_stepsize,
            );

            // Add the mutated state as a new sim-state.
            sim_states.0.push((final_state, summary));
        }

        view_state.i = common::increment_step(
            view_state.i,
            backward,
            sim_states.0.len() - 1,
            view_state.checkpoint_stepsize,
        );
        return;
    }

    let slowards = if keyboard_input.pressed(KeyCode::ArrowDown) {
        Some(true)
    } else if keyboard_input.pressed(KeyCode::ArrowUp) {
        Some(false)
    } else {
        None
    };
    if let Some(slowards) = slowards {
        let new_step = view_state.checkpoint_stepsize as i64 + (if slowards { -1 } else { 1 });
        view_state.checkpoint_stepsize = if new_step >= 1 { new_step as usize } else { 1 };
    }
}

#[derive(Debug, clap::Parser)]
#[command(name = "ricsek_view", about = "View a run results...")]
struct ViewCli {
    #[arg(short = 'w', long = "window-size", default_value = "800.0")]
    pub window_size: f64,

    #[arg(short = 'r', long = "run_id")]
    pub run_id: Option<usize>,
}

fn main() {
    env_logger::init();
    let args = ViewCli::parse();

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
        boundaries: Some(setup.parameters.boundaries.clone()),
    };

    info!("Got {} sim-states", sim_states.len());

    App::new()
        .add_plugins(DefaultPlugins.set(bevy::log::LogPlugin {
            level: bevy::log::Level::DEBUG,
            ..default()
        }))
        .add_plugins((LogDiagnosticsPlugin::default(), FrameTimeDiagnosticsPlugin))
        .insert_resource(env)
        .insert_resource(SimStates(sim_states))
        .insert_resource(SetupConfigRes(setup))
        .insert_resource(FlowViewState::default())
        .insert_resource(SimViewState::default())
        .add_systems(Startup, add_camera)
        .add_systems(Startup, add_axis_arrows)
        .add_systems(Startup, add_agents)
        .add_systems(Startup, add_flow_markers)
        .add_systems(Startup, environment::add_boundaries)
        .add_systems(PostStartup, flow::add_flow)
        .add_systems(Update, pan_orbit_camera_update)
        .add_systems(
            Update,
            (
                update_agent_points,
                update_agent_orientations,
                update_flow_markers,
                update_timestep.run_if(on_timer(Duration::from_secs_f64(TIME_STEP))),
            ),
        )
        .add_systems(
            PostUpdate,
            (
                flow::update_flow,
                flow::update_flow_markers.run_if(on_timer(Duration::from_secs_f64(TIME_STEP))),
            ),
        )
        .add_systems(Update, common::close_on_esc)
        .run();

    info!("Done!");
}
