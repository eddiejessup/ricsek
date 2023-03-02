use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use ricsek::common::*;

const PL: f64 = 700.0;

const TIME_STEP: f64 = 1.0 / 20.0;

#[derive(Component)]
struct Agent(usize);

#[derive(Component)]
struct Movable;

#[derive(Resource)]
struct ViewState {
    i: usize,
    last_update_i: Option<usize>,
}

#[derive(Resource)]
struct SimStates(Vec<SimState>);

#[derive(Resource)]
struct SimParamsRes(SimParams);

fn transform_coord(sd: f64, sl: f64) -> f32 {
    return (PL * sd / sl) as f32;
}

fn add_agents(
    mut commands: Commands,
    sim_states: Res<SimStates>,
    sim_params: Res<SimParamsRes>,
    view_state: Res<ViewState>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    commands.spawn(Camera2dBundle::default());

    let view_i = view_state.i;
    let cur_sim_state = &sim_states.0[view_i];

    let l = sim_params.0.l;

    for (i, r) in cur_sim_state.r.view().rows().into_iter().enumerate() {
        let pr_x = transform_coord(r[0], l);
        let pr_y = transform_coord(r[1], l);

        commands.spawn((
            MaterialMesh2dBundle {
                mesh: meshes.add(shape::RegularPolygon::new(2., 3).into()).into(),
                material: materials.add(ColorMaterial::from(Color::RED)),
                transform: Transform::from_translation(Vec3::new(pr_x, pr_y, 0.)),
                ..default()
            },
            Agent(i),
        ));
    }
}

fn update_agent_position(
    sim_states: Res<SimStates>,
    sim_params: Res<SimParamsRes>,
    mut view_state: ResMut<ViewState>,
    mut query: Query<(&mut Transform, &Agent)>,
) {
    let view_i = view_state.i;
    let cur_sim_state = &sim_states.0[view_i];
    let l = sim_params.0.l;

    match view_state.last_update_i {
        Some(last_update_i) => {
            if last_update_i == view_i {
                return;
            }
        }
        None => {}
    }
    println!("Updating positions for agents, to view_i: {}", view_i);
    for (mut transform, agent) in &mut query {
        let r = cur_sim_state.r.row(agent.0);
        transform.translation.x = transform_coord(r[0], l);
        transform.translation.y = transform_coord(r[1], l);
    }
    view_state.last_update_i = Some(view_i);
}
fn change_view(
    keyboard_input: Res<Input<KeyCode>>,
    sim_states: Res<SimStates>,
    mut view_state: ResMut<ViewState>,
) {
  let mut i = view_state.i;
    if keyboard_input.pressed(KeyCode::Left) && i > 0 {
      println!("Handling left");
      i -= 1;
    }

    if keyboard_input.pressed(KeyCode::Right) {
      println!("Handling right, {}", sim_states.0.len() - 1);
        i += 1;
        i = i.min(sim_states.0.len() - 1);
        println!("Now {}", i);
      }

    if i != view_state.i {
        println!("Changing view to i: {}", i);
    }

    view_state.i = i;
}

fn main() {
    let run_id: usize = 6;

    let conn = &mut ricsek::db::establish_connection();

    let sim_params = ricsek::db::read_run(conn, run_id);
    let sim_states = ricsek::db::read_run_sim_states(conn, run_id);

    println!("Got {} sim-states", sim_states.len());

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(SimStates(sim_states))
        .insert_resource(SimParamsRes(sim_params))
        .insert_resource(ViewState { i: 0, last_update_i: None })

        .add_startup_system(add_agents)
        .add_system_set(
          SystemSet::new()
              .with_run_criteria(bevy::time::FixedTimestep::step(TIME_STEP))
              .with_system(update_agent_position)
              .with_system(change_view)
      )
        .add_system(bevy::window::close_on_esc)
        .run();

    println!("Done!");
}
