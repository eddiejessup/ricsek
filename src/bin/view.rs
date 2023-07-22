use std::time::Duration;

use bevy::{prelude::*, time::common_conditions::on_timer};
use ricsek::view::*;

fn update_agent_position(
    sim_states: Res<SimStates>,
    env: Res<EnvironmentRes>,
    mut view_state: ResMut<ViewState>,
    mut query_ag: Query<(&mut Transform, &AgentId, Option<&AgentDirectionId>)>,
) {
    let view_i = view_state.i;
    let cur_sim_state = &sim_states.0[view_i];

    println!("Updating positions for agents, to view_i: {}", view_i);
    for (mut transform, agent_id, may_dir) in &mut query_ag {
        let z_off = match may_dir {
            Some(_) => 0.01,
            None => 0.0,
        };
        *transform = agent_transform(&env, &cur_sim_state.agents[agent_id.0], agent_id.0, z_off);
    }
    view_state.mark_fresh();
}
fn increment_step(cur_i: usize, backward: bool, maxi: usize) -> usize {
    if backward {
        match cur_i {
            0 => 0,
            _ => cur_i - 1,
        }
    } else {
        (cur_i + 1).min(maxi)
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

fn run_if_step_stale(view_state: Res<ViewState>) -> bool {
    view_state.is_stale()
}

fn main() {
    let conn = &mut ricsek::db::establish_connection();

    let run_id = ricsek::db::read_latest_run_id(conn);
    let sim_setup = ricsek::db::read_run(conn, run_id);
    let sim_states = ricsek::db::read_run_sim_states(conn, run_id);

    let env = EnvironmentRes {
        l: sim_setup.params.l,
        window_size: 800.0,
        arrow_length_pixels: 20.0,
    };

    println!("Got {} sim-states", sim_states.len());

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(env)
        .insert_resource(SimStates(sim_states))
        .insert_resource(SimSetupRes(sim_setup))
        .insert_resource(ViewState::new())
        .add_startup_system(add_obstacles)
        .add_startup_system(add_camera)
        .add_startup_system(add_agents)
        .add_system(change_view.run_if(on_timer(Duration::from_secs_f64(TIME_STEP))))
        .add_system(update_agent_position.run_if(run_if_step_stale))
        .add_system(bevy::window::close_on_esc)
        .run();

    println!("Done!");
}
