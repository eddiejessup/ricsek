use bevy::prelude::{Vec2, Vec3};
use bevy::render::mesh::{Indices, Mesh};
use bevy::render::render_asset::RenderAssetUsages;
use bevy::render::render_resource::PrimitiveTopology;

struct MeshData {
    positions: Vec<Vec3>,
    normals: Vec<Vec3>,
    uvs: Vec<Vec2>,
    indices: Vec<u32>,
}

impl MeshData {
    fn new(num_vertices: usize, num_indices: usize) -> Self {
        Self {
            positions: Vec::with_capacity(num_vertices),
            normals: Vec::with_capacity(num_vertices),
            uvs: Vec::with_capacity(num_vertices),
            indices: Vec::with_capacity(num_indices),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Cone {
    pub radius: f32,
    pub height: f32,
    pub segments: u32,
}

impl Default for Cone {
    fn default() -> Self {
        Cone {
            radius: 0.5,
            height: 1.0,
            segments: 32,
        }
    }
}

fn add_bottom(mesh: &mut MeshData, cone: &Cone) {
    let angle_step = std::f32::consts::TAU / cone.segments as f32;
    let base_index = mesh.positions.len() as u32;

    // Center
    let center_pos = Vec3::new(0.0, -cone.height / 2.0, 0.0);
    mesh.positions.push(center_pos);
    mesh.uvs.push(Vec2::new(0.5, 0.5));
    mesh.normals.push(-Vec3::Y);

    // Vertices
    for i in 0..=cone.segments {
        let theta = i as f32 * angle_step;
        let x_unit = f32::cos(theta);
        let z_unit = f32::sin(theta);

        let pos = Vec3::new(
            cone.radius * x_unit,
            -cone.height / 2.0,
            cone.radius * z_unit,
        );
        let uv = Vec2::new((z_unit * 0.5) + 0.5, (x_unit * -0.5) + 0.5);

        mesh.positions.push(pos);
        mesh.uvs.push(uv);
        mesh.normals.push(-Vec3::Y)
    }

    // Indices
    for i in 0..cone.segments {
        mesh.indices.push(base_index + i + 1);
        mesh.indices.push(base_index + i + 2);
        mesh.indices.push(base_index);
    }
}

fn add_body(mesh: &mut MeshData, cone: &Cone) {
    let angle_step = std::f32::consts::TAU / cone.segments as f32;
    let base_index = mesh.positions.len() as u32;

    // Add top vertices. We need to add multiple here because their normals differ
    for i in 0..cone.segments {
        let theta = i as f32 * angle_step + angle_step / 2.0;
        let x_unit = f32::cos(theta);
        let z_unit = f32::sin(theta);

        let slope = cone.radius / cone.height;
        let normal = Vec3::new(x_unit, slope, z_unit).normalize();

        mesh.positions.push(Vec3::new(0.0, cone.height / 2.0, 0.0));
        mesh.normals.push(normal);
        mesh.uvs.push(Vec2::new(0.5, 0.5));
    }

    // Add bottom vertices
    for i in 0..=cone.segments {
        let theta = i as f32 * angle_step;
        let x_unit = f32::cos(theta);
        let z_unit = f32::sin(theta);

        let slope = cone.radius / cone.height;
        let normal = Vec3::new(x_unit, slope, z_unit).normalize();

        let uv = Vec2::new((z_unit * 0.5) + 0.5, (x_unit * 0.5) + 0.5);

        mesh.positions.push(Vec3::new(
            x_unit * cone.radius,
            -cone.height / 2.0,
            z_unit * cone.radius,
        ));
        mesh.normals.push(normal);
        mesh.uvs.push(uv);
    }

    // Add indices
    for i in 0..cone.segments {
        let top = base_index + i;
        let left = base_index + cone.segments + i;
        let right = left + 1;

        mesh.indices.push(right);
        mesh.indices.push(left);
        mesh.indices.push(top);
    }
}

impl From<Cone> for Mesh {
    fn from(cone: Cone) -> Self {
        // Validate input parameters
        assert!(cone.height > 0.0, "Must have positive height");
        assert!(cone.radius > 0.0, "Must have positive radius");
        assert!(
            cone.segments > 2,
            "Must have at least 3 subdivisions to close the surface"
        );

        // code adapted from http://apparat-engine.blogspot.com/2013/04/procedural-meshes-torus.html
        // (source code at https://github.com/SEilers/Apparat)

        // bottom + body
        let n_vertices = (cone.segments + 2) + (cone.segments * 2 + 1);
        let n_triangles = cone.segments * 2;
        let n_indices = n_triangles * 3;

        let mut mesh = MeshData::new(n_vertices as usize, n_indices as usize);

        add_bottom(&mut mesh, &cone);
        add_body(&mut mesh, &cone);

        let mut m = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        );
        m.insert_indices(Indices::U32(mesh.indices));
        m.insert_attribute(Mesh::ATTRIBUTE_POSITION, mesh.positions);
        m.insert_attribute(Mesh::ATTRIBUTE_NORMAL, mesh.normals);
        m.insert_attribute(Mesh::ATTRIBUTE_UV_0, mesh.uvs);
        m
    }
}
