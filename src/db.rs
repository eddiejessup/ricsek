use crate::config::setup::SetupConfig;
use crate::geometry::line_segment::LineSegment;
use crate::state::{self};
use diesel::dsl::Eq;
use diesel::pg::PgConnection;
use diesel::prelude::*;
use diesel::result::Error;
use dotenvy::dotenv;
use nalgebra::Point3;
use std::env;
use time;

pub mod models;
pub mod schema;

pub fn establish_connection() -> PgConnection {
    dotenv().ok();

    let database_url = env::var("DATABASE_URL").expect("DATABASE_URL must be set");
    PgConnection::establish(&database_url)
        .unwrap_or_else(|_| panic!("Error connecting to {}", database_url))
}

pub fn initialize_run(conn: &mut PgConnection, config: &SetupConfig) -> usize {
    use crate::db::schema::run::dsl;

    let run_id: i32 = diesel::insert_into(crate::db::schema::run::table)
        .values((
            None::<Eq<dsl::id, i32>>,
            None::<Eq<dsl::created_at, time::OffsetDateTime>>,
            dsl::parameters.eq(serde_json::to_value(&config.parameters).unwrap()),
            dsl::agent_initialization
                .eq(serde_json::to_value(&config.agent_initialization).unwrap()),
            dsl::sampling_config.eq(serde_json::to_value(&config.sampling).unwrap()),
            dsl::sample_points.eq(serde_json::to_value(config.sample_points.clone()).unwrap()),
        ))
        .returning(dsl::id)
        .get_result(conn)
        .unwrap();
    run_id as usize
}

pub fn write_checkpoint(
    conn: &mut PgConnection,
    rid: usize,
    sim_state: &state::SimState,
    summary: &Option<state::StepSummary>,
) -> Result<i32, Error> {
    use crate::db::schema::env::dsl;

    let eid = conn.transaction::<i32, Error, _>(|conn| {
        // Debug.
        let eid: i32 = diesel::insert_into(dsl::env)
            .values((
                None::<Eq<dsl::id, i32>>,
                dsl::run_id.eq(rid as i32),
                dsl::step.eq(sim_state.step as i32),
                dsl::t.eq(sim_state.t),
                dsl::rng.eq(serde_json::to_value(sim_state.rng.clone()).unwrap()),
                dsl::step_summary.eq(summary.as_ref().map(|v| serde_json::to_value(v).unwrap())),
            ))
            .returning(dsl::id)
            .get_result(conn)?;

        use crate::db::schema::agent::dsl::*;
        let rows: Vec<_> = sim_state
            .agents
            .iter()
            .enumerate()
            .map(|(aid, a)| {
                (
                    agent_id.eq(aid as i32),
                    env_id.eq(eid),
                    r1x.eq(a.r1().x),
                    r1y.eq(a.r1().y),
                    r1z.eq(a.r1().z),
                    r2x.eq(a.r2().x),
                    r2y.eq(a.r2().y),
                    r2z.eq(a.r2().z),
                    th1.eq(a.th1),
                    th2.eq(a.th2),
                )
            })
            .collect();

        diesel::insert_into(agent).values(rows).execute(conn)?;
        Ok(eid)
    })?;

    Ok(eid)
}

pub fn read_latest_run_id(conn: &mut PgConnection) -> usize {
    use schema::run::dsl;
    let rid: i32 = dsl::run
        .order(dsl::created_at.desc())
        .select(dsl::id)
        .first(conn)
        .unwrap();
    rid as usize
}

pub fn read_run(conn: &mut PgConnection, rid: usize) -> SetupConfig {
    use schema::run::dsl;

    let v = dsl::run
        .filter(dsl::id.eq(rid as i32))
        .get_result::<models::Run>(conn)
        .unwrap();
    SetupConfig {
        parameters: serde_json::from_value(v.parameters).unwrap(),
        agent_initialization: serde_json::from_value(v.agent_initialization).unwrap(),
        sampling: serde_json::from_value(v.sampling_config).unwrap(),
        sample_points: serde_json::from_value(v.sample_points).unwrap(),
    }
}

pub fn read_run_envs(conn: &mut PgConnection, rid: usize) -> Vec<models::Env> {
    use schema::env::dsl;
    dsl::env
        .filter(dsl::run_id.eq(rid as i32))
        .order(dsl::step.asc())
        .load::<models::Env>(conn)
        .unwrap()
}

pub fn read_last_run_env(conn: &mut PgConnection, rid: usize) -> models::Env {
    use schema::env::dsl;
    dsl::env
        .filter(dsl::run_id.eq(rid as i32))
        .order(dsl::step.asc())
        .first(conn)
        .unwrap()
}

pub fn read_agents(conn: &mut PgConnection, env_id_: i32) -> Vec<state::Agent> {
    use schema::agent::dsl;

    let agent_vals: Vec<models::Agent> = dsl::agent
        .filter(dsl::env_id.eq(env_id_))
        .order(dsl::agent_id.asc())
        .get_results::<models::Agent>(conn)
        .unwrap();

    agent_vals
        .iter()
        .map(|a| state::Agent {
            seg: LineSegment {
                start: Point3::new(a.r1x, a.r1y, a.r1z),
                end: Point3::new(a.r2x, a.r2y, a.r2z),
            },
            th1: a.th1,
            th2: a.th2,
        })
        .collect()
}

pub fn env_to_sim_state_with_summary(
    conn: &mut PgConnection,
    env: &models::Env,
) -> state::SimStateWithSummary {
    let agents: Vec<state::Agent> = read_agents(conn, env.id);

    let sim_state = state::SimState {
        step: env.step as usize,
        t: env.t,
        agents,
        rng: serde_json::from_value(env.rng.clone()).unwrap(),
    };

    let step_summary: Option<state::StepSummary> = env
        .step_summary
        .as_ref()
        .map(|v| serde_json::from_value(v.clone()).unwrap());

    state::SimStateWithSummary {
        sim_state,
        step_summary,
    }
}

pub fn read_run_sim_states(conn: &mut PgConnection, rid: usize) -> Vec<state::SimStateWithSummary> {
    read_run_envs(conn, rid)
        .into_iter()
        .map(|env| env_to_sim_state_with_summary(conn, &env))
        .collect()
}

pub fn read_latest_checkpoint(conn: &mut PgConnection, rid: usize) -> state::SimStateWithSummary {
    let latest_env = read_last_run_env(conn, rid);
    env_to_sim_state_with_summary(conn, &latest_env)
}
