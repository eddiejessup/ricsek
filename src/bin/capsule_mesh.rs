use std::f64::consts::PI;

use log::info;
use nalgebra::{DMatrix, DVector, Point3, UnitVector3, Vector3};
use ricsek::{
    geometry::line_segment::BoundingBox,
    view::{
        common::{add_axis_arrows, point3_to_gvec3, spawn_arrow, vec3_to_gvec3},
        environment::Environment,
        pan_orbit_camera::{add_camera, pan_orbit_camera_update},
        *,
    },
};
use tobj::{self, LoadOptions};

use bevy::prelude::*;
use bevy_obj::ObjPlugin;

#[derive(Debug, Clone)]
pub struct Triangle {
    pub p0: Point3<f64>,
    pub p1: Point3<f64>,
    pub p2: Point3<f64>,
    normal: UnitVector3<f64>,
}

#[derive(Resource)]
pub struct TriangleSet {
    pub triangles: Vec<(Triangle, Vector3<f64>)>,
}

// const MESH_PATH: &str = "meshes/plane_10x10.obj";
// const MESH_PATH: &str = "meshes/sphere.obj";
// const MESH_PATH: &str = "meshes/sphere_5seg.obj";
// const MESH_PATH: &str = "meshes/capsule.obj";
const MESH_PATH: &str = "meshes/capsule_ico_seg5.obj";

impl Triangle {
    pub fn new(p0: Point3<f64>, p1: Point3<f64>, p2: Point3<f64>) -> Self {
        let a = p1 - p0;
        let b = p2 - p0;
        let normal = UnitVector3::new_normalize(a.cross(&b));
        Triangle { p0, p1, p2, normal }
    }

    pub fn centroid(&self) -> Point3<f64> {
        ((self.p0.coords + self.p1.coords + self.p2.coords) / 3.0).into()
    }

    pub fn normal(&self) -> UnitVector3<f64> {
        self.normal
    }

    pub fn area(&self) -> f64 {
        let a = self.p1 - self.p0;
        let b = self.p2 - self.p0;
        0.5 * a.cross(&b).norm()
    }

    pub fn bounding_box(&self) -> BoundingBox {
        BoundingBox {
            min: Point3::new(
                self.p0.x.min(self.p1.x).min(self.p2.x),
                self.p0.y.min(self.p1.y).min(self.p2.y),
                self.p0.z.min(self.p1.z).min(self.p2.z),
            ),
            max: Point3::new(
                self.p0.x.max(self.p1.x).max(self.p2.x),
                self.p0.y.max(self.p1.y).max(self.p2.y),
                self.p0.z.max(self.p1.z).max(self.p2.z),
            ),
        }
    }
}

fn read_triangle_mesh(path: &str) -> (Vec<Point3<f64>>, Vec<Point3<usize>>, Vec<Triangle>) {
    let (models, _) = tobj::load_obj(
        path,
        &LoadOptions {
            single_index: false,
            triangulate: false,
            ignore_points: true,
            ignore_lines: true,
        },
    )
    .unwrap();
    let mesh = &models[0].mesh;
    info!(
        "Number of positions, 3 * num vertices: {} (implies n vertices: {})",
        mesh.positions.len(),
        mesh.positions.len() / 3
    );
    info!(
        "Number of normals, 3 * num vertices and then some: {} (extra: {})",
        mesh.normals.len(),
        mesh.normals.len() - mesh.positions.len()
    );
    info!("Number of face indices, 3 * num faces, to store index for 3 vertex indices: {} (implies n faces: {})", mesh.indices.len(), mesh.indices.len() / 3);
    info!(
        "Number of face normal_indices, 3 * num faces, to store index for 3 normal indices: {}",
        mesh.normal_indices.len()
    );

    let vertex_positions = &mesh.positions;
    let mut vertices = Vec::new();
    // Traverse the vertices
    for vx_ix in 0..(vertex_positions.len() / 3) {
        let vx_pos_ix = 3 * vx_ix;
        vertices.push(Point3::new(
            vertex_positions[vx_pos_ix] as f64,
            vertex_positions[vx_pos_ix + 1] as f64,
            vertex_positions[vx_pos_ix + 2] as f64,
        ));
    }

    let vx_position_indices = &mesh.indices;
    let mut faces = Vec::new();
    let mut face_ixs = Vec::new();
    // Traverse the faces
    for face_ix in 0..(vx_position_indices.len() / 3) {
        let vx_0_ix = 3 * face_ix;
        let vx_1_ix = 3 * face_ix + 1;
        let vx_2_ix = 3 * face_ix + 2;

        let vx_0_pos_ix = vx_position_indices[vx_0_ix] as usize;
        let vx_1_pos_ix = vx_position_indices[vx_1_ix] as usize;
        let vx_2_pos_ix = vx_position_indices[vx_2_ix] as usize;

        face_ixs.push(Point3::new(vx_0_pos_ix, vx_1_pos_ix, vx_2_pos_ix));
        faces.push(Triangle::new(
            vertices[vx_0_pos_ix],
            vertices[vx_1_pos_ix],
            vertices[vx_2_pos_ix],
        ));
    }

    (vertices, face_ixs, faces)
}

fn add_mesh(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    faces: Res<TriangleSet>,
    env: Res<Environment>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let mesh_material = materials.add(StandardMaterial::from(Color::RED.with_a(0.4)));
    let normal_material = materials.add(StandardMaterial::from(Color::BLUE));
    let value_material = materials.add(StandardMaterial::from(Color::GREEN));
    let mesh_handle = asset_server.load(MESH_PATH);
    commands.spawn((PbrBundle {
        mesh: mesh_handle,
        material: mesh_material,
        transform: Transform::IDENTITY.with_scale(Vec3::splat(1.0)),
        ..default()
    },));

    let f = 1.0;
    let g = 0.1;

    // Max magnitude.
    let v_mag_max = faces
        .triangles
        .iter()
        .map(|(_f, v)| v.norm())
        .fold(std::f64::NEG_INFINITY, |acc, v_mag| acc.max(v_mag));

    for (face, value) in faces.triangles.iter() {
        let centroid = face.centroid();
        let normal = face.normal();

        // Normal.
        match vec3_to_gvec3(&normal.into_inner()).try_normalize() {
            Some(glam_u) => {
                // Normal.
                commands
                    .spawn(SpatialBundle {
                        transform: Transform::from_translation(point3_to_gvec3(&centroid))
                            .with_scale(Vec3::splat((env.arrow_length * g) as f32))
                            .looking_to(glam_u, Vec3::Z),
                        ..default()
                    })
                    .with_children(|parent| {
                        spawn_arrow(parent, &mut meshes, normal_material.clone());
                    });
            }
            None => {}
        };

        // Vector value.
        match vec3_to_gvec3(&value).try_normalize() {
            Some(glam_u) => {
                // Normal.
                let v_mag = value.norm() / v_mag_max;
                commands
                    .spawn(SpatialBundle {
                        transform: Transform::from_translation(point3_to_gvec3(&centroid))
                            .with_scale(Vec3::splat((env.arrow_length * f * v_mag) as f32))
                            .looking_to(glam_u, Vec3::Z),
                        ..default()
                    })
                    .with_children(|parent| {
                        spawn_arrow(parent, &mut meshes, value_material.clone());
                    });
            }
            None => {}
        };
    }
}

fn main() {
    env_logger::init();

    info!("Loading mesh");
    let (_mesh_vertices, _mesh_face_ixs, mesh_faces) =
        read_triangle_mesh(&format!("{}/{}", "assets", MESH_PATH));
    // let mut triangles_wtr = csv::Writer::from_path("triangles.csv").unwrap();
    // for face_ix in mesh_face_ixs {
    //     triangles_wtr
    //         .write_record(&[
    //             face_ix.x.to_string(),
    //             face_ix.y.to_string(),
    //             face_ix.z.to_string(),
    //         ])
    //         .unwrap();
    // }
    // triangles_wtr.flush().unwrap();

    // let mut vertex_wtr = csv::Writer::from_path("vertices.csv").unwrap();
    // for vertex in mesh_vertices {
    //     vertex_wtr
    //         .write_record(&[
    //             vertex.x.to_string(),
    //             vertex.y.to_string(),
    //             vertex.z.to_string(),
    //         ])
    //         .unwrap();
    // }
    // vertex_wtr.flush().unwrap();

    info!("Creating matrix");
    let n_components = 3;
    let v_axis = 1;
    let v_mag = 1.0;

    let mut v_inf = DVector::repeat(n_components, 0.0);
    v_inf[v_axis] = v_mag;

    // Add equations from inf boundary and object boundary. Concat them.

    let mut vels_desired = Vec::new();
    for _ in 0..mesh_faces.len() {
        for j in 0..n_components {
            vels_desired.push(-v_inf[j]);
        }
    }

    let n_eqns = n_components * mesh_faces.len();
    let mut elem_stress_vel_coeff_matrix = DMatrix::zeros(n_eqns, n_eqns);
    let viscosity = 0.001;
    // let viscosity = 0.1;
    let global_g_coeff = 1.0 / (8.0 * PI * viscosity);
    // let global_t_coeff = 1.0 / (8.0 * PI);
    let radius = 1.0;

    let mut stress_ideal = DVector::repeat(n_components, 0.0);
    stress_ideal[v_axis] = -(3.0 / 2.0) * viscosity * v_mag / radius;

    let mut elem_stresses_ideal = Vec::new();
    for (focus_ix, face) in mesh_faces.iter().enumerate() {
        let face_centroid = face.centroid();
        for (other_ix, other_face) in mesh_faces.iter().enumerate() {
            if focus_ix == other_ix {
                continue;
            }
            let r = face_centroid - other_face.centroid();

            let g_matrix = ricsek::dynamics::stokes_solutions::stokeslet_matrix(r);
            // let t_matrix = ricsek::dynamics::stokes_solutions::stokeslet_stress_matrix(r);
            let coeff_matrix = global_g_coeff * g_matrix * other_face.area();

            // let other_normal = other_face.normal();

            // focus_component indicates the component of the focus point.
            for focus_component in 0..n_components {
                let focus_ix_component = n_components * focus_ix + focus_component;

                // let mut t_contribution = 0.0;
                // i indicates the component of the target
                for other_component in 0..n_components {
                    let other_ix_component = n_components * other_ix + other_component;
                    // for k in 0..n_components {
                    // t_contribution += v_inf[other_component] * t_matrix[other_component][focus_component][k] * other_normal[k];
                    // }
                    elem_stress_vel_coeff_matrix[(focus_ix_component, other_ix_component)] =
                        coeff_matrix[(focus_component, other_component)];
                }
                // vels_desired[focus_ix_component] += global_t_coeff * t_contribution;
            }
        }
        for j in 0..n_components {
            elem_stresses_ideal.push(stress_ideal[j]);
        }
    }
    let vels_desired = DVector::from_vec(vels_desired);
    let elem_stresses_ideal = DVector::from_vec(elem_stresses_ideal);

    info!("Solving matrix");
    // let elem_stresses_computed = elem_stress_vel_coeff_matrix.clone().svd(true, true).solve(&vels_desired, 0.1).expect("Solve failed.");
    let elem_stresses_computed = elem_stress_vel_coeff_matrix
        .clone()
        .lu()
        .solve(&vels_desired)
        .expect("Solve failed.");

    info!("Computing solution");
    let vels_pred = elem_stress_vel_coeff_matrix.clone() * elem_stresses_computed.clone();
    let vels_ideal_pred = elem_stress_vel_coeff_matrix.clone() * elem_stresses_ideal.clone();

    info!("Writing data");

    let mut stresses = Vec::new();
    for i in 0..mesh_faces.len() {
        let mut elem_stress_vec = Vec::new();
        for j in 0..n_components {
            elem_stress_vec.push(elem_stresses_computed[n_components * i + j]);
        }
        for _ in n_components..3 {
            elem_stress_vec.push(0.0);
        }
        stresses.push(DVector::from_vec(elem_stress_vec));
    }

    // Write results as CSV for visualization.
    // let mut matrix_wtr = csv::Writer::from_path("matrix.csv").unwrap();
    // for a_row in elem_stress_vel_coeff_matrix.row_iter() {
    //     let column = a_row.iter().map(|x| x.to_string()).collect::<Vec<String>>();
    //     matrix_wtr
    //         .write_record(&column)
    //         .unwrap();
    // }
    // matrix_wtr.flush().unwrap();

    let mut velocities_wtr = csv::Writer::from_path("velocities.csv").unwrap();
    for ((vel_desired, vel_pred), vel_ideal_pred) in vels_desired
        .iter()
        .zip(vels_pred.iter())
        .zip(vels_ideal_pred.iter())
    {
        velocities_wtr
            .write_record([
                &vel_desired.to_string(),
                &vel_pred.to_string(),
                &vel_ideal_pred.to_string(),
            ])
            .unwrap();
    }
    velocities_wtr.flush().unwrap();

    let mut stress_wtr = csv::Writer::from_path("stresses.csv").unwrap();
    for stress in stresses.clone() {
        let mut record = Vec::new();
        for j in 0..n_components {
            record.push(stress[j].to_string());
        }
        record.push(stress.norm().to_string());
        for j in 0..n_components {
            record.push(stress_ideal[j].to_string());
        }
        record.push(stress_ideal.norm().to_string());
        stress_wtr.write_record(&record).unwrap();
    }
    stress_wtr.flush().unwrap();

    // Compute total force and torque across body.
    let mut total_force = Vector3::zeros();
    let mut total_torque = Vector3::zeros();
    for (face, stress) in mesh_faces.iter().zip(stresses.iter()) {
        let face_centroid = face.centroid();
        let moment_arm = face_centroid - Point3::origin();
        let force = Vector3::new(stress[0], 0.0, 0.0) * face.area();
        total_force += force;
        let torque = moment_arm.cross(&force);
        total_torque += torque;
    }
    info!("Total force: {}", total_force);
    info!("Total torque: {}", total_torque);

    info!("Drawing");
    let env = Environment {
        boundaries: None,
        arrow_length: 0.025,
    };

    // Zip mesh_faces with stresses.
    let triangles = mesh_faces
        .iter()
        .zip(stresses.iter())
        .map(|(face, stress)| (face.clone(), Vector3::new(stress[0], 0.0, 0.0)))
        .collect::<Vec<(Triangle, Vector3<f64>)>>();

    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(ObjPlugin)
        .insert_resource(env)
        .insert_resource(TriangleSet { triangles })
        .add_systems(Startup, add_camera)
        .add_systems(Startup, environment::add_boundaries)
        .add_systems(Startup, add_axis_arrows)
        .add_systems(Startup, add_mesh)
        .add_systems(Update, pan_orbit_camera_update)
        .add_systems(Update, bevy::window::close_on_esc)
        .run();
}
