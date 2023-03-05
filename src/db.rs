use crate::common::*;
use crate::common::params::*;
use crate::math::*;
use diesel::pg::PgConnection;
use diesel::prelude::*;
use dotenvy::dotenv;
use ndarray::prelude::*;
use ndarray::Zip;
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
    segment_vals: &Vec<LineSegment>,
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
    let rows = Zip::indexed(sim_state.r.view().rows())
        .and(sim_state.u_p.view().rows())
        .map_collect(|aid, r, u_p| {
            (
                agent_id.eq(aid as i32),
                env_id.eq(eid),
                rx.eq(r[0]),
                ry.eq(r[1]),
                ux.eq(u_p[0]),
                uy.eq(u_p[1]),
            )
        })
        .into_raw_vec();

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
    let (sim_params_json, sim_segments_json) = run
        .filter(schema::run::id.eq(rid as i32))
        .select((params, segments))
        .get_result::<(serde_json::Value, serde_json::Value)>(conn)
        .unwrap();
    let params_val = serde_json::from_value(sim_params_json).unwrap();
    let segments_val = serde_json::from_value(sim_segments_json).unwrap();
    SimSetup {
        params: params_val,
        segments: segments_val,
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

    let rx_arr = Array1::from_iter(agent_vals.iter().map(|a| a.rx));
    let ry_arr = Array1::from_iter(agent_vals.iter().map(|a| a.ry));
    let ux_arr = Array1::from_iter(agent_vals.iter().map(|a| a.ux));
    let uy_arr = Array1::from_iter(agent_vals.iter().map(|a| a.uy));

    let r = ndarray::stack(Axis(1), &[rx_arr.view(), ry_arr.view()]).unwrap();
    let u_p = ndarray::stack(Axis(1), &[ux_arr.view(), uy_arr.view()]).unwrap();

    SimState {
        step: env.step as usize,
        t: env.t,
        r,
        u_p,
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
