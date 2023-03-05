use crate::common::params::*;
use crate::common::*;
use diesel::pg::PgConnection;
use diesel::prelude::*;
use dotenvy::dotenv;
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

pub fn initialize_run(
    conn: &mut PgConnection,
    sim_params: &SimParams,
    segment_vals: &Vec<geo::Line>,
) -> usize {
    use crate::db::schema::run::dsl::*;
    use diesel::dsl::Eq;
    let run_id: i32 = diesel::insert_into(crate::db::schema::run::table)
        .values((
            None::<Eq<id, i32>>,
            None::<Eq<created_at, time::OffsetDateTime>>,
            params.eq(serde_json::to_value(sim_params).unwrap()),
            segments.eq(serde_json::to_value(segment_vals).unwrap()),
        ))
        .returning(id)
        .get_result(conn)
        .unwrap();
    run_id as usize
}

pub fn write_checkpoint(conn: &mut PgConnection, rid: usize, sim_state: &SimState) -> i32 {
    use crate::db::schema::env::dsl::*;
    use diesel::dsl::Eq;
    let eid: i32 = diesel::insert_into(env)
        .values((
            None::<Eq<id, i32>>,
            run_id.eq(rid as i32),
            step.eq(sim_state.step as i32),
            t.eq(sim_state.t),
        ))
        .returning(id)
        .get_result(conn)
        .unwrap();

    use crate::db::schema::agent::dsl::*;
    let rows: Vec<_> = sim_state
        .agents
        .iter()
        .enumerate()
        .map(|(aid, a)| {
            (
                agent_id.eq(aid as i32),
                env_id.eq(eid),
                rx.eq(a.r.x()),
                ry.eq(a.r.y()),
                ux.eq(a.u.x()),
                uy.eq(a.u.y()),
            )
        })
        .collect();

    diesel::insert_into(agent)
        .values(rows)
        .execute(conn)
        .unwrap();
    eid
}

pub fn read_latest_run_id(conn: &mut PgConnection) -> usize {
    use schema::run::dsl::*;
    let rid: i32 = run.order(created_at.desc()).select(id).first(conn).unwrap();
    rid as usize
}

pub fn read_run(conn: &mut PgConnection, rid: usize) -> SimSetup {
    use schema::run::dsl::*;
    let v = run
        .filter(schema::run::id.eq(rid as i32))
        .get_result::<models::Run>(conn)
        .unwrap();
    SimSetup {
        params: serde_json::from_value(v.params).unwrap(),
        segments: serde_json::from_value(v.segments).unwrap(),
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
            r: (a.rx, a.ry).into(),
            u: (a.ux, a.uy).into(),
        })
        .collect();

    SimState {
        step: env.step as usize,
        t: env.t,
        agents,
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
