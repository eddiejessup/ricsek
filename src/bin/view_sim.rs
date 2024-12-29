use std::time::Duration;

use bevy::{
    color::palettes::css,
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    prelude::*,
    render::mesh::SphereKind,
    time::common_conditions::on_timer,
};
use clap::Parser;
use nalgebra::Vector3;
use num_traits::zero;
use ricsek::{
    config::{
        run::RunContext,
        setup::{parameters::singularities::SingularityParams, SetupConfig},
    },
    dynamics::run_steps,
    state::{SimStateWithSummary, StepSummary},
    view::{
        common::*,
        environment::Environment,
        flow::FlowViewState,
        pan_orbit_camera::{add_camera, pan_orbit_camera_update},
        *,
    },
};

// Resources.

#[derive(Resource)]
struct SimViewState {
    pub setup_config: SetupConfig,
    pub states: Vec<SimStateWithSummary>,
    pub i: usize,
    pub checkpoint_stepsize: usize,
    pub sim_stepsize: usize,
}

impl SimViewState {
    pub fn new(setup_config: SetupConfig, states: Vec<SimStateWithSummary>) -> Self {
        Self {
            setup_config,
            states,
            i: 0,
            checkpoint_stepsize: 1,
            sim_stepsize: 10,
        }
    }

    pub fn active_state(&self) -> SimStateWithSummary {
        self.states[self.i].clone()
    }

    pub fn latest_state(&self) -> SimStateWithSummary {
        self.states[self.states.len() - 1].clone()
    }

    pub fn n_states(&self) -> usize {
        self.states.len()
    }

    pub fn max_i(&self) -> usize {
        self.states.len() - 1
    }

    pub fn max_t(&self) -> f64 {
        self.latest_state().sim_state.t
    }

    pub fn max_step(&self) -> usize {
        self.latest_state().sim_state.step
    }

    pub fn at_latest_state(&self) -> bool {
        self.i == self.max_i()
    }

    pub fn increment_step(&mut self, backward: bool) -> usize {
        self.i = common::increment_step(self.i, backward, self.max_i(), self.checkpoint_stepsize);
        self.i
    }

    pub fn go_to_first_state(&mut self) {
        self.i = 0;
    }

    pub fn go_to_latest_state(&mut self) {
        self.i = self.max_i();
    }

    pub fn simulate_new_state(&mut self) {
        let mut latest_state = (self.latest_state()).sim_state;

        let run_context = RunContext::new(
            &self.setup_config.parameters,
            latest_state.agents.len(),
            self.setup_config.sample_points.clone(),
        );

        let step_summary = run_steps(
            &self.setup_config.parameters,
            &mut latest_state,
            &run_context,
            self.sim_stepsize,
        );

        let new_entry = SimStateWithSummary {
            sim_state: latest_state,
            step_summary: Some(step_summary),
        };
        self.states.push(new_entry);
    }

    pub fn update_checkpoint_stepsize(&mut self, slowards: bool) {
        let new_step = self.checkpoint_stepsize as i64 + (if slowards { -1 } else { 1 });
        self.checkpoint_stepsize = if new_step >= 1 { new_step as usize } else { 1 };
    }
}

/// Helper resource for tracking our asset
// #[derive(Resource)]
// struct GltfAsset(Handle<Gltf>);

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
struct SimUIText;

#[derive(Component)]
struct ForceMarker;

#[derive(Component)]
struct ForceArrow;

#[derive(Component)]
pub struct SimFlowMarkerId(pub usize);

// Systems.

// Startup.

fn add_sim_ui_text(
    parent: &mut ChildBuilder,
    text_styles: (TextFont, TextLayout),
    label: &str,
    name: String,
) {
    parent
        .spawn((
            Text::new(format!("{}: ", label)),
            text_styles.clone(),
            TextColor(Color::from(css::MINT_CREAM)),
        ))
        .with_child((
            SimUIText,
            Name::new(name),
            TextSpan::new("Unknown"),
            text_styles.clone(),
            TextColor(Color::from(css::GHOST_WHITE)),
        ));
}

fn add_sim_ui(mut commands: Commands, asset_server: Res<AssetServer>) {
    let text_styles = (
        TextFont {
            font: asset_server.load("fonts/Inconsolata-Regular.ttf"),
            font_size: 15.0,
            ..default()
        },
        TextLayout::new_with_justify(JustifyText::Left),
    );
    // First spawn a node to display each text in one column on the right.
    commands
        .spawn((
            BackgroundColor(Color::from(css::DARK_SLATE_GRAY)),
            Node {
                display: Display::Flex,
                width: Val::Px(300.0),
                border: UiRect::all(Val::Px(2.)),
                flex_direction: FlexDirection::Column,
                padding: UiRect::all(Val::Px(5.)),
                row_gap: Val::Px(5.0),
                margin: UiRect::left(Val::Auto),
                ..default()
            },
        ))
        .with_children(|parent| {
            for (label, name) in [
                ("Time", "time_display"),
                ("Step", "step_display"),
                ("Checkpoint step", "checkpoint_step_display"),
                ("Checkpoint step size", "checkpoint_step_size"),
            ] {
                add_sim_ui_text(parent, text_styles.clone(), label, name.to_string());
            }
        });
}

fn add_agents(
    mut commands: Commands,
    view_state: Res<SimViewState>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cur_sim_state = &view_state.active_state().sim_state;

    let params = &view_state.setup_config.parameters;

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

fn add_sim_flow_markers(
    mut commands: Commands,
    view_state: Res<SimViewState>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Marker bases.
    let red: Handle<StandardMaterial> =
        materials.add(StandardMaterial::from(Color::from(css::RED)));
    let cube: Handle<Mesh> = meshes.add(Cuboid::from_length(0.2).mesh().build());

    commands.spawn_batch(
        view_state
            .setup_config
            .sample_points
            .iter()
            .cloned()
            .enumerate()
            .map(move |(i, sample)| {
                (
                    Mesh3d(cube.clone()),
                    MeshMaterial3d(red.clone()),
                    Transform::from_translation(point3_to_gvec3(&sample)),
                    SimFlowMarkerId(i),
                    flow::VectorSet(vec![]),
                )
            })
            .collect::<Vec<_>>(),
    );
}

// Update.

fn update_sim_ui(
    view_state: Res<SimViewState>,
    mut q_text_spans: Query<(&Name, &mut TextSpan), With<SimUIText>>,
) {
    let cur_sim_state = &view_state.active_state().sim_state;

    for (name, mut text_span) in &mut q_text_spans {
        match name.as_str() {
            "time_display" => {
                text_span.0 = format!("{:.2}s / {:.2}s", cur_sim_state.t, view_state.max_t());
            }
            "step_display" => {
                text_span.0 = format!("{} / {}", cur_sim_state.step, view_state.max_step());
            }
            "checkpoint_step_display" => {
                text_span.0 = format!("{} / {}", view_state.i, view_state.max_i());
            }
            "checkpoint_step_size" => {
                text_span.0 = format!("{}", view_state.checkpoint_stepsize);
            }
            _ => {}
        };
    }
}

fn update_agent_points(
    view_state: Res<SimViewState>,
    mut q_point: Query<(&AgentId, &mut flow::VectorSet, &mut Transform)>,
) {
    let active_state = &view_state.active_state();
    let cur_sim_state = &active_state.sim_state;

    for (agent_id, mut vector_set, mut transform) in &mut q_point {
        let agent = &cur_sim_state.agents[agent_id.0];
        *transform = Transform::from_translation(point3_to_gvec3(&agent.seg.centroid()));

        if let Some(StepSummary {
            agent_summaries,
            singularities: _,
            fluid_flow: _,
        }) = &active_state.step_summary
        {
            let agent_summary = &agent_summaries[agent_id.0];
            *vector_set = flow::VectorSet(vec![
                (
                    "f_agent_electro".to_string(),
                    agent_summary.f_agent_electro.scale(F_COEFF),
                ),
                (
                    "torque_agent_electro".to_string(),
                    agent_summary.torque_agent_electro.scale(TORQUE_COEFF),
                ),
                (
                    "f_propulsion".to_string(),
                    agent_summary.f_propulsion.scale(F_COEFF),
                ),
                (
                    "f_boundary_electro".to_string(),
                    agent_summary.f_boundary_electro.scale(F_COEFF),
                ),
                (
                    "torque_boundary_electro".to_string(),
                    agent_summary.torque_boundary_electro.scale(TORQUE_COEFF),
                ),
                ("v_fluid".to_string(), agent_summary.v_fluid.scale(V_COEFF)),
            ])
        }
    }
}

fn update_agent_orientations(
    view_state: Res<SimViewState>,
    mut q_ag: Query<(&AgentId, &mut Transform, &AgentBodyComponent)>,
) {
    let cur_sim_state = &view_state.active_state().sim_state;
    debug!("Updating orientations for agents");
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

fn update_sim_flow_markers(
    view_state: Res<SimViewState>,
    mut q_point: Query<(&SimFlowMarkerId, &mut flow::VectorSet)>,
) {
    let cur_step_summary = &view_state.active_state().step_summary;

    if let Some(cur_step_summary) = cur_step_summary {
        for (marker_id, mut vector_set) in &mut q_point {
            let marker_fluid_v = cur_step_summary.fluid_flow[marker_id.0];
            *vector_set = flow::VectorSet(vec![
                ("f_agent_electro".to_string(), zero()),
                ("torque_agent_electro".to_string(), zero()),
                ("f_propulsion".to_string(), zero()),
                ("f_boundary_electro".to_string(), zero()),
                ("torque_boundary_electro".to_string(), zero()),
                ("v_fluid".to_string(), marker_fluid_v.scale(V_COEFF)),
            ])
        }
    }
}
// Forces.
fn add_forces(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // We don't know how many singularities there will be, so just add a bunch of empty entities.
    for _ in 0..1000 {
        commands
            .spawn((ForceMarker, Transform::default(), Visibility::Hidden))
            .with_children(|marker_parent| {
                marker_parent
                    .spawn((ForceArrow, Transform::default(), Visibility::default()))
                    .with_children(|arrow_parent| {
                        let arrow_material: Handle<StandardMaterial> = materials.add(
                            StandardMaterial::from(Color::from(css::RED).with_alpha(0.9)),
                        );
                        common::spawn_arrow(arrow_parent, &mut meshes, arrow_material);
                    });
            });
    }
}

fn update_forces(
    view_state: Res<SimViewState>,
    mut q_force_marker: Query<(&mut Transform, &mut Visibility, &Children), With<ForceMarker>>,
    mut q_force_arrow: Query<(&mut Transform, &mut Visibility), (With<ForceArrow>, Without<ForceMarker>)>,
) {
    if let Some(ss) = &view_state.active_state().step_summary {
        let singularities = &ss.singularities;
        // Go over all force markers, and set them to represent the singularity.
        // If we don't have enough, just ignore the rest.
        // If there are fewer singularities than force markers, hide the rest.
        for (marker_i, (mut marker_transform, mut marker_visibility, children)) in
            &mut q_force_marker.iter_mut().enumerate()
        {
            if marker_i < singularities.len() {
                let singularity = &singularities[marker_i];
                *marker_transform =
                    Transform::from_translation(point3_to_gvec3(&singularity.0.position));
                *marker_visibility = Visibility::Inherited;

                // Now update the arrow to point in the direction of the force.
                let force_vector = match &singularity.1 {
                    SingularityParams::Stokeslet { a } => a,
                    _ => {
                        panic!("Unsupported singularity type");
                    }
                };

                for &marker_child in children.iter() {
                    let (mut arrow_transform, mut arrow_visibility) =
                        match q_force_arrow.get_mut(marker_child) {
                            Ok(x) => x,
                            Err(_) => continue,
                        };

                    info!("Setting singularity force arrow to {:?}", force_vector);
                    flow::set_rot_and_viz(
                        &mut arrow_transform,
                        &mut arrow_visibility,
                        &force_vector,
                        1e-2,
                    );
                }
            } else {
                *marker_visibility = Visibility::Hidden;
            }
        }
    }
}

fn update_sim_state(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut view_state: ResMut<SimViewState>,
) {
    if keyboard_input.just_pressed(KeyCode::Digit0) {
        view_state.go_to_first_state();
        return;
    } else if keyboard_input.just_pressed(KeyCode::Digit9) {
        view_state.go_to_latest_state();
        return;
    } else {
        let backward = if keyboard_input.pressed(KeyCode::ArrowLeft) {
            Some(true)
        } else if keyboard_input.pressed(KeyCode::ArrowRight) {
            Some(false)
        } else {
            None
        };

        if let Some(backward) = backward {
            if !backward && view_state.at_latest_state() {
                view_state.simulate_new_state();
            }

            view_state.increment_step(backward);
        }
    }

    // Update the checkpoint stepsize.
    let slowards = if keyboard_input.pressed(KeyCode::ArrowDown) {
        Some(true)
    } else if keyboard_input.pressed(KeyCode::ArrowUp) {
        Some(false)
    } else {
        None
    };
    if let Some(slowards) = slowards {
        view_state.update_checkpoint_stepsize(slowards);
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

    let sim_states_with_summaries = ricsek::db::read_run_sim_states(conn, run_id);

    let env = Environment {
        boundaries: Some(setup.parameters.boundaries.clone()),
    };

    info!("Got {} sim-states", sim_states_with_summaries.len());

    App::new()
        .add_plugins(DefaultPlugins.set(bevy::log::LogPlugin {
            level: bevy::log::Level::DEBUG,
            ..default()
        }))
        .add_plugins((LogDiagnosticsPlugin::default(), FrameTimeDiagnosticsPlugin))
        .insert_resource(env)
        .insert_resource(FlowViewState::default())
        .insert_resource(SimViewState::new(setup, sim_states_with_summaries))
        .add_systems(
            Startup,
            (
                add_camera,
                add_sim_ui,
                add_axis_arrows,
                add_agents,
                add_forces,
                add_sim_flow_markers,
                environment::add_boundaries,
            ),
        )
        .add_systems(PostStartup, flow::add_flow_vectors)
        .add_systems(
            Update,
            (
                pan_orbit_camera_update,
                update_agent_points,
                update_agent_orientations,
                update_sim_flow_markers,
                update_sim_state.run_if(on_timer(Duration::from_secs_f64(TIME_STEP))),
                common::close_on_esc,
            ),
        )
        .add_systems(
            PostUpdate,
            (
                flow::update_flow_vectors,
                update_sim_ui,
                update_forces,
                flow::update_flow_view_state.run_if(on_timer(Duration::from_secs_f64(TIME_STEP))),
            ),
        )
        .run();

    info!("Done!");
}
