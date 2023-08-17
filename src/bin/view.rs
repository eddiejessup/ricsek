use std::{f32::consts::FRAC_PI_2, time::Duration};

use bevy::{prelude::*, time::common_conditions::on_timer};
use ricsek::view::{
    common::nalgebra_to_glam_vec,
    environment::Environment,
    flow::FlowViewState,
    pan_orbit_camera::{add_camera_startup, pan_orbit_camera_update},
    *,
};
use structopt::StructOpt;

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

    let materials = [
        materials.add(StandardMaterial::from(Color::RED)),
        materials.add(StandardMaterial::from(Color::GREEN)),
        materials.add(StandardMaterial::from(Color::BLUE)),
        materials.add(StandardMaterial::from(Color::PURPLE)),
        materials.add(StandardMaterial::from(Color::CYAN)),
    ];

    let agent_mesh = meshes.add(
        shape::UVSphere {
            radius: env.transform_coord(radius),
            ..default()
        }
        .into(),
    );
    let cone: Handle<Mesh> = meshes.add(
        bevy_more_shapes::Cone {
            radius: 1.0,
            height: 1.0,
            segments: 32,
        }
        .into(),
    );
    let cylinder_height = 3.0;
    let cylinder: Handle<Mesh> = meshes.add(
        shape::Cylinder {
            radius: 0.3,
            height: cylinder_height,
            resolution: 32,
            ..default()
        }
        .into(),
    );

    let transform_mesh = Transform::from_rotation(Quat::from_rotation_x(-FRAC_PI_2));

    cur_sim_state.agents.iter().enumerate().for_each(|(i, _a)| {
        let material = materials[i % 5].clone();
        commands
            .spawn((
                SpatialBundle::default(),
                AgentId(i),
                flow::VectorSet(vec![]),
            ))
            .with_children(|parent| {
                parent.spawn(PbrBundle {
                    mesh: agent_mesh.clone(),
                    material: material.clone(),
                    transform: transform_mesh,
                    ..default()
                });
                parent.spawn(PbrBundle {
                    mesh: cone.clone(),
                    material: material.clone(),
                    transform: transform_mesh.with_translation(-Vec3::Z * cylinder_height),
                    ..default()
                });
                parent.spawn(PbrBundle {
                    mesh: cylinder.clone(),
                    material: material.clone(),
                    transform: transform_mesh.with_translation(-Vec3::Z * cylinder_height / 2.0),
                    ..default()
                });
            });
    });
}

fn update_agents(
    sim_states: Res<SimStates>,
    env: Res<Environment>,
    view_state: Res<ViewState>,
    mut query_ag: Query<(&AgentId, &mut Transform, &mut flow::VectorSet)>,
) {
    let view_i = view_state.i;
    let cur_sim_state = &sim_states.0[view_i];
    let opt_summary = &cur_sim_state.summary;
    match opt_summary {
        None => println!("No agent summary for view_i: {}", view_i),
        Some(_) => println!("Agent summary for view_i: {}", view_i),
    };

    println!("Updating positions for agents, to view_i: {}", view_i);
    for (agent_id, mut transform, mut vector_set) in &mut query_ag {
        let agent = &cur_sim_state.agents[agent_id.0];
        *transform = Transform::from_translation(env.transformed_vec3(agent.r))
            .looking_to(nalgebra_to_glam_vec(&agent.u), Vec3::Z);

        if let Some(agent_summaries) = opt_summary {
            let agent_summary = &agent_summaries[agent_id.0];
            *vector_set = flow::VectorSet(vec![
                ("agent_electro".to_string(), agent_summary.v_agent_electro),
                ("agent_hydro".to_string(), agent_summary.v_agent_hydro),
                ("propulsion".to_string(), agent_summary.v_propulsion),
                (
                    "boundary_electro".to_string(),
                    agent_summary.v_boundary_electro,
                ),
                ("singularity".to_string(), agent_summary.v_singularity),
            ])
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
    let args = ViewCli::from_args();

    let conn = &mut ricsek::db::establish_connection();

    let run_id = match args.run_id {
        Some(run_id) => run_id,
        None => {
            println!("No run_id specified, using latest run_id");
            ricsek::db::read_latest_run_id(conn)
        }
    };

    let setup = ricsek::db::read_run(conn, run_id);
    let sim_states = ricsek::db::read_run_sim_states(conn, run_id);

    let env = Environment {
        boundaries: setup.parameters.boundaries.clone(),
        arrow_length: 7.0e-6,
        length_factor: 1e6,
    };

    println!("Got {} sim-states", sim_states.len());

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(env)
        .insert_resource(SimStates(sim_states))
        .insert_resource(SetupRes(setup))
        .insert_resource(FlowViewState::new(10000))
        .insert_resource(ViewState::default())
        .add_systems(Startup, (add_camera_startup, add_agents))
        .add_systems(Startup, environment::add_environment)
        .add_systems(PostStartup, flow::add_flow)
        .add_systems(Update, pan_orbit_camera_update)
        .add_systems(
            Update,
            (
                flow::update_flow,
                flow::change_view.run_if(on_timer(Duration::from_secs_f64(TIME_STEP))),
            ),
        )
        .add_systems(
            Update,
            (
                update_agents,
                change_view.run_if(on_timer(Duration::from_secs_f64(TIME_STEP))),
            ),
        )
        .add_systems(Update, bevy::window::close_on_esc)
        .run();

    println!("Done!");
}
