use crate::config::setup::parameters::common::BoundaryConfig;
use bevy::prelude::*;

#[derive(Resource)]
pub struct Environment {
    pub boundaries: Option<BoundaryConfig>,
    pub arrow_length: f64,
}

pub fn add_boundaries(
    mut commands: Commands,
    env: Res<Environment>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let b = match &env.boundaries {
        None => return,
        Some(b) => b,
    };

    let l = b.l();
    // Need 6 rectangles, 2 for each boundary of the environment.
    // For the boundary along x, the rectangle should have width l.y and height l.z
    // And should be centred at (l.x / 2, 0, 0) and (-l.x / 2, 0, 0)
    // And so on.
    // The plane is on the XY plane, so we must do some rotations:
    // - For the planes along the x-direction, it should sit on the YZ plane, so rotate by 90 degrees around Z.
    let axis_configs = [
        (
            Vec2 {
                x: l.z as f32,
                y: l.y as f32,
            },
            Vec3::X,
            l.x,
            b.0.x.closed,
            Vec3::Y,
        ),
        (
            Vec2 {
                x: l.z as f32,
                y: l.x as f32,
            },
            Vec3::Y,
            l.y,
            b.0.y.closed,
            Vec3::X,
        ),
        (
            Vec2 {
                x: l.x as f32,
                y: l.y as f32,
            },
            Vec3::Z,
            l.z,
            b.0.y.closed,
            Vec3::Y,
        ),
    ];

    for (size, axis, axis_l, closed, up) in axis_configs {
        for sgn in [1, -1] {
            commands.spawn((PbrBundle {
                mesh: meshes.add(shape::Quad { size, flip: false }.into()),
                material: materials.add(StandardMaterial::from(if closed {
                    Color::WHITE
                } else {
                    Color::YELLOW
                })),
                transform: Transform::from_translation(axis * (sgn as f32) * axis_l as f32 / 2.0)
                    .looking_to(axis * sgn as f32, up),
                ..default()
            },));
        }
    }
}
