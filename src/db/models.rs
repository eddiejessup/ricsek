use time;

use diesel::prelude::*;

#[derive(Queryable)]
pub struct Env {
    pub id: usize,
    pub run_id: usize,
    pub step: usize,
    pub t: f64,
}

#[derive(Queryable)]
pub struct Agent {
    pub agent_id: usize,
    pub env_id: usize,
    pub rx: f64,
    pub ry: f64,
    pub ux: f64,
    pub uy: f64,
    pub vx: f64,
    pub vy: f64,
}

#[derive(Queryable)]
pub struct Run {
    pub id: i32,
    pub created_at: time::OffsetDateTime,
    pub params: serde_json::Value,
}
