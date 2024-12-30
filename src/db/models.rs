use time;

use diesel::prelude::*;

#[derive(Queryable)]
pub struct Env {
    pub id: i32,
    pub run_id: i32,
    pub step: i32,
    pub t: f64,
    pub rng: serde_json::Value,
    pub step_summary: Option<serde_json::Value>,
}

#[derive(Queryable)]
pub struct Agent {
    pub agent_id: i32,
    pub env_id: i32,
    pub r1x: f64,
    pub r1y: f64,
    pub r1z: f64,
    pub r2x: f64,
    pub r2y: f64,
    pub r2z: f64,
    pub tail_phase: f64,
}

#[derive(Queryable)]
pub struct Run {
    pub id: i32,
    pub created_at: time::OffsetDateTime,
    pub parameters: serde_json::Value,
    pub agent_initialization: serde_json::Value,
    pub sampling_config: serde_json::Value,
    pub sample_points: serde_json::Value,
}
