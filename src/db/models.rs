use time;

use diesel::prelude::*;

#[derive(Queryable)]
pub struct Env {
    pub id: i32,
    pub run_id: i32,
    pub step: i32,
    pub t: f64,
}

#[derive(Queryable)]
pub struct Agent {
    pub agent_id: i32,
    pub env_id: i32,
    pub rx: f64,
    pub ry: f64,
    pub rz: f64,
    pub ux: f64,
    pub uy: f64,
    pub uz: f64,
}

#[derive(Queryable)]
pub struct Run {
    pub id: i32,
    pub created_at: time::OffsetDateTime,
    pub parameters: serde_json::Value,
    pub agent_initialization: serde_json::Value,
}
