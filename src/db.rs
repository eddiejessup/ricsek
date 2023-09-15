use crate::config::setup::SetupConfig;
use crate::geometry::line_segment::LineSegment;
use crate::state::*;
use diesel::pg::PgConnection;
use diesel::prelude::*;
use dotenvy::dotenv;
use nalgebra::Point3;
use std::env;
use diesel::result::Error;
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
    use crate::db::schema::run::dsl::*;
    use diesel::dsl::Eq;
    let run_id: i32 = diesel::insert_into(crate::db::schema::run::table)
        .values((
            None::<Eq<id, i32>>,
            None::<Eq<created_at, time::OffsetDateTime>>,
            parameters.eq(serde_json::to_value(&config.parameters).unwrap()),
            agent_initialization.eq(serde_json::to_value(&config.agent_initialization).unwrap()),
        ))
        .returning(id)
        .get_result(conn)
        .unwrap();
    run_id as usize
}

pub fn write_checkpoint(
    conn: &mut PgConnection,
    rid: usize,
    sim_state: &SimState,
    summary: Option<Vec<AgentStepSummary>>,
) -> Result<i32, Error> {
    use crate::db::schema::env::dsl::*;
    use diesel::dsl::Eq;

    let eid = conn.transaction::<i32, Error, _>(|conn| {

    let eid: i32 = diesel::insert_into(env)
        .values((
            None::<Eq<id, i32>>,
            run_id.eq(rid as i32),
            step.eq(sim_state.step as i32),
            t.eq(sim_state.t),
        ))
        .returning(id)
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
                step_summary.eq(summary
                    .as_ref()
                    .map(|ss| serde_json::to_value(&ss[aid]).unwrap())),
            )
        })
        .collect();

    diesel::insert_into(agent)
        .values(rows)
        .execute(conn)?;
    Ok(eid)
    })?;

    Ok(eid)
}

pub fn read_latest_run_id(conn: &mut PgConnection) -> usize {
    use schema::run::dsl::*;
    let rid: i32 = run.order(created_at.desc()).select(id).first(conn).unwrap();
    rid as usize
}

pub fn read_run(conn: &mut PgConnection, rid: usize) -> SetupConfig {
    use schema::run::dsl::*;
    let v = run
        .filter(schema::run::id.eq(rid as i32))
        .get_result::<models::Run>(conn)
        .unwrap();
    SetupConfig {
        parameters: serde_json::from_value(v.parameters).unwrap(),
        agent_initialization: serde_json::from_value(v.agent_initialization).unwrap(),
    }
}

pub fn read_run_envs(conn: &mut PgConnection, rid: usize) -> Vec<models::Env> {
    use schema::env::dsl::*;
    env.filter(run_id.eq(rid as i32))
        .order(step.asc())
        .load::<models::Env>(conn)
        .unwrap()
}

pub fn env_to_sim_state(conn: &mut PgConnection, env: &models::Env) -> SimState {
    use schema::agent::dsl::*;

    let agent_vals: Vec<models::Agent> = agent
        .filter(env_id.eq(env.id))
        .order(agent_id.asc())
        .get_results::<models::Agent>(conn)
        .unwrap();

    let agents: Vec<Agent> = agent_vals
        .iter()
        .map(|a| Agent {
            seg: LineSegment {
              start: Point3::new(a.r1x, a.r1y, a.r1z),
              end: Point3::new(a.r2x, a.r2y, a.r2z),
            },
            th1: a.th1,
            th2: a.th2,
        })
        .collect();

    let summary: Option<Vec<AgentStepSummary>> = agent_vals
        .iter()
        .map(|a| {
            a.step_summary
                .as_ref()
                .and_then(|ss| serde_json::from_value(ss.clone()).ok())
        })
        .collect();

    SimState {
        step: env.step as usize,
        t: env.t,
        agents,
        summary,
    }
}

pub fn read_run_sim_states(conn: &mut PgConnection, rid: usize) -> Vec<SimState> {
    read_run_envs(conn, rid)
        .into_iter()
        .map(|env| env_to_sim_state(conn, &env))
        .collect()
}

pub fn read_latest_checkpoint(conn: &mut PgConnection, rid: usize) -> SimState {
    let envs = read_run_envs(conn, rid);
    let latest_env = envs.last().unwrap();
    env_to_sim_state(conn, latest_env)
}
