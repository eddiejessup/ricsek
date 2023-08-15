use std::time::Duration;

use bevy::{prelude::*, time::common_conditions::on_timer};
use ricsek::view::{
    environment::Environment,
    pan_orbit_camera::{add_camera_startup, pan_orbit_camera_update},
    *,
};
use structopt::StructOpt;

fn update_agent_position(
    sim_states: Res<SimStates>,
    env: Res<Environment>,
    view_state: Res<ViewState>,
    mut query_ag: Query<(&mut Transform, &AgentId)>,
) {
    let view_i = view_state.i;
    let cur_sim_state = &sim_states.0[view_i];

    println!("Updating positions for agents, to view_i: {}", view_i);
    for (mut transform, agent_id) in &mut query_ag {
        *transform = env.transformed_agent(&cur_sim_state.agents[agent_id.0]);
    }
}

fn change_view(
    keyboard_input: Res<Input<KeyCode>>,
    sim_states: Res<SimStates>,
    mut view_state: ResMut<ViewState>,
) {
    let backward = if keyboard_input.pressed(KeyCode::Left) {
        Some(true)
    } else if keyboard_input.pressed(KeyCode::Right) {
        Some(false)
    } else {
        None
    };

    if let Some(backward) = backward {
        view_state.i = increment_step(view_state.i, backward, sim_states.0.len() - 1);
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
        boundaries: setup.parameters.sim_params.boundaries.clone(),
        arrow_length: 20.0e-6,
        length_factor: 1e6,
    };

    println!("Got {} sim-states", sim_states.len());

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(env)
        .insert_resource(SimStates(sim_states))
        .insert_resource(SetupRes(setup))
        .insert_resource(ViewState::new())
        .add_systems(Startup, (add_camera_startup, add_agents))
        .add_systems(Startup, add_environment)
        .add_systems(Update, pan_orbit_camera_update)
        .add_systems(
            Update,
            (
                change_view.run_if(on_timer(Duration::from_secs_f64(TIME_STEP))),
                bevy::window::close_on_esc,
                update_agent_position,
            ),
        )
        .run();

    println!("Done!");
}
