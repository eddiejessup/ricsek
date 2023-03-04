use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use ndarray::Zip;
use ricsek::common::*;

const PL: f64 = 1000.0;

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
struct SimSetupRes(SimSetup);

fn transform_coord(sd: f64, sl: f64) -> f32 {
    (PL * sd / sl) as f32
}

fn add_agents(
    mut commands: Commands,
    sim_states: Res<SimStates>,
    sim_setup: Res<SimSetupRes>,
    view_state: Res<ViewState>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    commands.spawn(Camera2dBundle::default());

    let view_i = view_state.i;
    let cur_sim_state = &sim_states.0[view_i];

    let sim_setup1 = &sim_setup.0;

    let l = sim_setup1.params.l;

    Zip::indexed(cur_sim_state.r.view().rows())
        .and(cur_sim_state.u_p.view().rows())
        .for_each(|i, r, u_p| {
            let pr_x = transform_coord(r[0], l);
            let pr_y = transform_coord(r[1], l);

            let color = match i % 5 {
                0 => Color::RED,
                1 => Color::GREEN,
                2 => Color::BLUE,
                3 => Color::PURPLE,
                4 => Color::CYAN,
                _ => Color::WHITE,
            };
            let rect_shape = Vec2 { x: 10.0, y: 1.0 };

            let angle_to_x = u_p[1].atan2(u_p[0]);
            let transform = Transform::IDENTITY
                .with_translation(Vec3::new(pr_x, pr_y, 0.))
                .with_rotation(Quat::from_rotation_z(angle_to_x as f32));
            commands.spawn((
                MaterialMesh2dBundle {
                    mesh: meshes.add(shape::Quad::new(rect_shape).into()).into(),
                    material: materials.add(ColorMaterial::from(color)),
                    transform,
                    ..default()
                },
                Agent(i),
            ));
        });

    for seg in sim_setup1.segments.iter() {
        let centre = seg.centre();
        let width_render = 1.0;

        let rect_shape = Vec2 {
            x: transform_coord(seg.length(), l),
            y: width_render,
        };

        let transform = Transform::IDENTITY
            .with_translation(Vec3::new(
                transform_coord(centre.x, l),
                transform_coord(centre.y, l),
                0.0,
            ))
            .with_rotation(Quat::from_rotation_z(seg.angle_to_x() as f32));

        commands.spawn((MaterialMesh2dBundle {
            mesh: meshes.add(shape::Quad::new(rect_shape).into()).into(),
            material: materials.add(ColorMaterial::from(Color::YELLOW)),
            transform,
            ..default()
        },));
    }
}

fn update_agent_position(
    sim_states: Res<SimStates>,
    sim_setup: Res<SimSetupRes>,
    mut view_state: ResMut<ViewState>,
    mut query: Query<(&mut Transform, &Agent)>,
) {
    let view_i = view_state.i;
    let cur_sim_state = &sim_states.0[view_i];
    let l = sim_setup.0.params.l;

    if let Some(last_update_i) = view_state.last_update_i {
        if last_update_i == view_i {
            return;
        }
    }
    println!("Updating positions for agents, to view_i: {}", view_i);
    for (mut transform, agent) in &mut query {
        let r = cur_sim_state.r.row(agent.0);
        transform.translation.x = transform_coord(r[0], l);
        transform.translation.y = transform_coord(r[1], l);

        let u_p = cur_sim_state.u_p.row(agent.0);
        transform.rotation = Quat::from_rotation_z(array_angle_to_x(u_p) as f32);
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
    let conn = &mut ricsek::db::establish_connection();

    let run_id = ricsek::db::read_latest_run_id(conn);
    let sim_setup = ricsek::db::read_run(conn, run_id);
    let sim_states = ricsek::db::read_run_sim_states(conn, run_id);

    println!("Got {} sim-states", sim_states.len());

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(SimStates(sim_states))
        .insert_resource(SimSetupRes(sim_setup))
        .insert_resource(ViewState {
            i: 0,
            last_update_i: None,
        })
        .add_startup_system(add_agents)
        .add_system_set(
            SystemSet::new()
                .with_run_criteria(bevy::time::FixedTimestep::step(TIME_STEP))
                .with_system(update_agent_position)
                .with_system(change_view),
        )
        .add_system(bevy::window::close_on_esc)
        .run();

    println!("Done!");
}
