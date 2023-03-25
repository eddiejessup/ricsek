use crate::{
    math::array_angle_to_x,
    parameters::SimSetup,
    state::{Agent, SimState},
};
use bevy::{prelude::*, render::render_resource::PrimitiveTopology, sprite::MaterialMesh2dBundle};

const PL: f64 = 1000.0;

pub const TIME_STEP: f64 = 1.0 / 40.0;

#[derive(Component)]
pub struct AgentId(pub usize);

#[derive(Component)]
pub struct AgentDirectionId(pub usize);

#[derive(Component)]
pub struct Movable;

#[derive(Resource)]
pub struct ViewState {
    pub i: usize,
    pub rendered_i: Option<usize>,
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

pub fn transform_coord(sd: f64, sl: f64) -> f32 {
    (PL * sd / sl) as f32
}

pub fn invert_coord(sd: f32, sl: f64) -> f64 {
    sl * (sd as f64) / PL
}

pub fn agent_transform(a: &Agent, l: f64, i: usize, z_offset: f32) -> Transform {
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
        .with_translation(Vec3::new(
            transform_coord(a.r.x(), l),
            transform_coord(a.r.y(), l),
            z,
        ))
        .with_rotation(Quat::from_rotation_z(array_angle_to_x(a.u.into()) as f32))
}

pub fn add_camera(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());
}

pub fn add_agents(
    mut commands: Commands,
    sim_states: Res<SimStates>,
    sim_setup: Res<SimSetupRes>,
    view_state: Res<ViewState>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let view_i = view_state.i;
    let cur_sim_state = &sim_states.0[view_i];

    let sim_setup = &sim_setup.0;

    let l = sim_setup.params.l;
    let radius = sim_setup.params.ag_radius;

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
                            radius: transform_coord(radius, l),
                            vertices: 10,
                        })
                        .into(),
                    )
                    .into(),
                material: materials.add(ColorMaterial::from(color)),
                transform: agent_transform(a, l, i, 0.0),
                ..default()
            },
            AgentId(i),
        ));

        commands.spawn((
            MaterialMesh2dBundle {
                mesh: meshes.add(arrow(10.0, 1.0)).into(),
                material: materials.add(ColorMaterial::from(Color::RED)),
                // Add small amount to 'z' translation to avoid overlap
                transform: agent_transform(a, l, i, 0.01),
                ..default()
            },
            AgentId(i),
            AgentDirectionId(i),
        ));
    });
}

pub fn add_obstacles(
    mut commands: Commands,
    sim_setup: Res<SimSetupRes>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let sim_setup = &sim_setup.0;

    let l = sim_setup.params.l;
    let z: f32 = 0.0;
    for cap in sim_setup.capsules.iter() {
        let centre = cap.centroid();

        let cap_shape = Vec2 {
            x: transform_coord(cap.segment_length(), l),
            y: transform_coord(cap.width(), l),
        };

        let transform = Transform::IDENTITY
            .with_translation(Vec3::new(
                transform_coord(centre.x(), l),
                transform_coord(centre.y(), l),
                z,
            ))
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
                        radius: transform_coord(cap.radius, l),
                        vertices: 100,
                    }
                    .into(),
                )
                .into(),
            material: materials.add(ColorMaterial::from(Color::YELLOW)),
            transform: Transform::IDENTITY.with_translation(Vec3::new(
                transform_coord(cap.segment.start.x, l),
                transform_coord(cap.segment.start.y, l),
                z,
            )),
            ..default()
        });
        commands.spawn(MaterialMesh2dBundle {
            mesh: meshes
                .add(
                    shape::Circle {
                        radius: transform_coord(cap.radius, l),
                        vertices: 100,
                    }
                    .into(),
                )
                .into(),
            material: materials.add(ColorMaterial::from(Color::YELLOW)),
            transform: Transform::IDENTITY.with_translation(Vec3::new(
                transform_coord(cap.segment.end.x, l),
                transform_coord(cap.segment.end.y, l),
                z,
            )),
            ..default()
        });
    }
}

pub fn arrow(s: f32, ar: f32) -> Mesh {
    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);

    // Define the vertices of the arrow
    // 2               4
    // 1     0         2
    // 0                   6
    // 1     1         3
    // 2               5
    //
    //       0 1 2 3 4 5 6 7
    // References are as looking along the shaft towards the head.
    let vertices: Vec<Vec3> = [
        // bottom-left of shaft.
        [-0.0, 0.1, 0.0],
        // bottom-right of shaft.
        [-0.0, -0.1, 0.0],
        // top-left of shaft.
        [0.5, 0.1, 0.0],
        // top-right of shaft.
        [0.5, -0.1, 0.0],
        // top-left of head.
        [0.5, 0.2, 0.0],
        // top-right of head.
        [0.5, -0.2, 0.0],
        // head tip.
        [0.7, 0.0, 0.0],
    ]
    .iter()
    .map(|v| {
        let mut vec3 = Vec3::from(*v) * s;
        vec3.y *= ar;
        vec3
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
    sim_setup: Res<SimSetupRes>,
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
        let l = sim_setup.0.params.l;

        eprintln!("Pixel coords: {}/{}", world_position.x, world_position.y);
        eprintln!(
            "Sim coords: {}/{}",
            invert_coord(world_position.x, l),
            invert_coord(world_position.y, l)
        );
    }
}
