#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]

use nalgebra::{Point3, Vector3};

use crate::{config::setup::parameters::common, geometry::point};

include!(concat!(env!("OUT_DIR"), "/cuda_ricsek_bindings.rs"));

pub fn zero_v3() -> V3 {
    V3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }
}

pub fn zero_v3_pair() -> V3Pair {
    V3Pair {
        a: zero_v3(),
        b: zero_v3(),
    }
}

pub fn vec_as_V3(p: Vector3<f64>) -> V3 {
    V3 {
        x: p.x as f32,
        y: p.y as f32,
        z: p.z as f32,
    }
}

pub fn point_as_V3(p: Point3<f64>) -> V3 {
    V3 {
        x: p.x as f32,
        y: p.y as f32,
        z: p.z as f32,
    }
}

pub fn v3_as_vec(v: V3) -> Vector3<f64> {
    Vector3::new(v.x as f64, v.y as f64, v.z as f64)
}

pub fn boundary_config_to_c(bc: &common::BoundaryConfig) -> BoundaryConfig {
    BoundaryConfig {
        lengths: vec_as_V3(bc.l()),
        closed_x: bc.0.x.closed,
        closed_y: bc.0.y.closed,
        closed_z: bc.0.z.closed,
    }
}

pub fn object_point_to_c(op: &point::ObjectPoint) -> ObjectPoint {
    ObjectPoint {
        object_id: op.object_id,
        position_com: point_as_V3(op.position_com),
        position: point_as_V3(op.position),
    }
}
