use crate::{
    parameters::SimSetup,
    state::{Agent, SimState},
    math::capsule::Capsule,
};
use bevy::{prelude::*, render::render_resource::PrimitiveTopology, sprite::MaterialMesh2dBundle};
use nalgebra::{Point2, Vector2};

pub const TIME_STEP: f64 = 1.0 / 40.0;

#[derive(Component)]
pub struct AgentId(pub usize);

#[derive(Component)]
pub struct AgentDirectionId(pub usize);

#[derive(Resource)]
pub struct Obstacles(pub Vec<Capsule>);

#[derive(Component)]
pub struct Movable;

#[derive(Resource)]
pub struct ViewState {
    pub i: usize,
    pub rendered_i: Option<usize>,
}

#[derive(Resource)]
pub struct EnvironmentRes {
    pub l: f64,
    pub window_size: f64,
    pub arrow_length_pixels: f64,
}

impl EnvironmentRes {
    pub fn transform_coord(&self, sd: f64) -> f32 {
        (self.l / 2.0 + (self.window_size * sd / self.l)) as f32
    }

    pub fn transformed_vec3(&self, sd: Point2<f64>, z: f32) -> Vec3 {
        Vec3::new(self.transform_coord(sd.x), self.transform_coord(sd.y), z)
    }

    pub fn transformed_vec2(&self, sd: Point2<f64>) -> Vec2 {
        Vec2::new(self.transform_coord(sd.x), self.transform_coord(sd.y))
    }
}

impl ViewState {
    pub fn new() -> Self {
        Self {
            i: 0,
            rendered_i: None,
        }
    }

    pub fn is_stale(&self) -> bool {
        match self.rendered_i {
            Some(i) => self.i != i,
            None => true,
        }
    }

    pub fn mark_fresh(&mut self) {
        self.rendered_i = Some(self.i);
    }
}

#[derive(Resource)]
pub struct SimStates(pub Vec<SimState>);

#[derive(Resource)]
pub struct SimSetupRes(pub SimSetup);

pub fn invert_coord(sd: f32, sl: f64, pl: f64) -> f64 {
    sl * (sd as f64) / pl
}

pub fn agent_transform(env: &EnvironmentRes, a: &Agent, i: usize, z_offset: f32) -> Transform {
    let z = z_offset
        + match i % 5 {
            0 => 0.1,
            1 => 0.2,
            2 => 0.3,
            3 => 0.4,
            4 => 0.5,
            _ => 0.6,
        };
    Transform::IDENTITY
        .with_translation(env.transformed_vec3(a.r, z))
        .with_rotation(Quat::from_rotation_z(a.u.angle(&Vector2::x()) as f32))
}

pub fn add_camera(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());
}

pub fn add_agents(
    mut commands: Commands,
    sim_states: Res<SimStates>,
    env: Res<EnvironmentRes>,
    sim_setup: Res<SimSetupRes>,
    view_state: Res<ViewState>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let view_i = view_state.i;
    let cur_sim_state = &sim_states.0[view_i];

    let sim_setup = &sim_setup.0;

    let radius = sim_setup.params.agent_radius;

    cur_sim_state.agents.iter().enumerate().for_each(|(i, a)| {
        let color = match i % 5 {
            0 => Color::RED,
            1 => Color::GREEN,
            2 => Color::BLUE,
            3 => Color::PURPLE,
            4 => Color::CYAN,
            _ => Color::WHITE,
        };

        commands.spawn((
            MaterialMesh2dBundle {
                mesh: meshes
                    .add(
                        (shape::Circle {
                            radius: env.transform_coord(radius),
                            vertices: 10,
                        })
                        .into(),
                    )
                    .into(),
                material: materials.add(ColorMaterial::from(color)),
                transform: agent_transform(&env, a, i, 0.0),
                ..default()
            },
            AgentId(i),
        ));

        commands.spawn((
            MaterialMesh2dBundle {
                mesh: meshes.add(arrow(1.0)).into(),
                material: materials.add(ColorMaterial::from(Color::RED)),
                // Add small amount to 'z' translation to avoid overlap
                transform: agent_transform(&env, a, i, 0.01).with_scale(Vec3::splat(10.0)),
                ..default()
            },
            AgentId(i),
            AgentDirectionId(i),
        ));
    });
}

pub fn add_obstacles(
    mut commands: Commands,
    env: Res<EnvironmentRes>,
    sim_setup: Res<SimSetupRes>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let z: f32 = 0.0;
    for cap in &sim_setup.0.capsules {
        let centre = cap.centroid();

        let cap_shape = Vec2 {
            x: env.transform_coord(cap.segment_length()),
            y: env.transform_coord(cap.width()),
        };

        let transform = Transform::IDENTITY
            .with_translation(env.transformed_vec3(centre, z))
            .with_rotation(Quat::from_rotation_z(cap.angle_to_x() as f32));

        commands.spawn(MaterialMesh2dBundle {
            mesh: meshes.add(shape::Quad::new(cap_shape).into()).into(),
            material: materials.add(ColorMaterial::from(Color::YELLOW)),
            transform,
            ..default()
        });
        commands.spawn(MaterialMesh2dBundle {
            mesh: meshes
                .add(
                    shape::Circle {
                        radius: env.transform_coord(cap.radius),
                        vertices: 100,
                    }
                    .into(),
                )
                .into(),
            material: materials.add(ColorMaterial::from(Color::YELLOW)),
            transform: Transform::IDENTITY
                .with_translation(env.transformed_vec3(cap.start_point(), z)),
            ..default()
        });
        commands.spawn(MaterialMesh2dBundle {
            mesh: meshes
                .add(
                    shape::Circle {
                        radius: env.transform_coord(cap.radius),
                        vertices: 100,
                    }
                    .into(),
                )
                .into(),
            material: materials.add(ColorMaterial::from(Color::YELLOW)),
            transform: Transform::IDENTITY
                .with_translation(env.transformed_vec3(cap.end_point(), z)),
            ..default()
        });
    }
}

pub fn arrow(thickness: f64) -> Mesh {
    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);

    // Define the vertices of the arrow

    //   y
    //   ^
    // 4 |               4
    // 3 |
    // 2 |
    // 1 |     0         2
    // 0 |                     6
    // 1 |     1         3
    // 2 |
    // 3 |
    // 4 |               5
    //    ---------------------> x
    //         0 1 2 3 4 5 6 7 8
    // References are as looking along the shaft towards the head.
    let head_width = 4.0;
    let shaft_length = 4.0;
    let shaft_width = 0.75;
    let head_length = 3.0;
    let vertices: Vec<Vec3> = [
        // 0: bottom-left of shaft.
        [0.0, shaft_width],
        // 1: bottom-right of shaft.
        [0.0, -shaft_width],
        // 2: top-left of shaft.
        [shaft_length, shaft_width],
        // 3: top-right of shaft.
        [shaft_length, -shaft_width],
        // 4: top-left of head.
        [shaft_length, head_width],
        // 5: top-right of head.
        [shaft_length, -head_width],
        // 6: head tip.
        [shaft_length + head_length, 0.0],
    ]
    .iter()
    .map(|v| Vec3 {
        x: ((v[0]) / (shaft_length + head_length)) as f32,
        y: (thickness * (v[1]) / (shaft_length + head_length)) as f32,
        z: 0.0,
    })
    .collect();

    // Define the indices of the triangles that make up the arrow
    let indices = bevy::render::mesh::Indices::U32(vec![0, 1, 2, 1, 3, 2, 4, 5, 6]);

    // Add the vertices and indices to the mesh
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, vertices);
    mesh.set_indices(Some(indices));

    mesh
}

pub fn cursor_system(
    env: Res<EnvironmentRes>,
    // need to get window dimensions
    window_query: Query<&Window, With<bevy::window::PrimaryWindow>>,
    // query to get camera transform
    camera_q: Query<(&Camera, &GlobalTransform)>,
) {
    // get the camera info and transform
    // assuming there is exactly one main camera entity, so query::single() is OK
    let (camera, camera_transform) = camera_q.single();

    let Ok(window) = window_query.get_single() else {
    return;
  };

    // check if the cursor is inside the window and get its position
    // then, ask bevy to convert into world coordinates, and truncate to discard Z
    if let Some(world_position) = window
        .cursor_position()
        .and_then(|cursor| camera.viewport_to_world(camera_transform, cursor))
        .map(|ray| ray.origin.truncate())
    {
        eprintln!("Pixel coords: {}/{}", world_position.x, world_position.y);
        eprintln!(
            "Sim coords: {}/{}",
            invert_coord(world_position.x, env.l, env.window_size),
            invert_coord(world_position.y, env.l, env.window_size)
        );
    }
}
