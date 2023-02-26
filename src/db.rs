use crate::common::*;
use diesel::pg::PgConnection;
use diesel::prelude::*;
use dotenvy::dotenv;
use ndarray::Zip;
use ndarray::prelude::*;
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

pub fn initialize_run(conn: &mut PgConnection, sim_params: &SimParams) -> usize {
    use crate::db::schema::run::dsl::*;
    use diesel::dsl::Eq;
    let run_id: i32 = diesel::insert_into(crate::db::schema::run::table)
        .values((
            None::<Eq<id, i32>>,
            None::<Eq<created_at, time::OffsetDateTime>>,
            params.eq(serde_json::to_value(&sim_params).unwrap()),
        ))
        .returning(id)
        .get_result(conn)
        .unwrap();
    return run_id as usize;
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
    return eid;
}

pub fn read_latest_checkpoint(conn: &mut PgConnection, rid: usize) -> (SimParams, SimState) {
    use schema::{agent::dsl::*, env::dsl::*, run::dsl::*};

    let sim_params_json = run
        .filter(schema::run::id.eq(rid as i32))
        .select(params)
        .get_result::<serde_json::Value>(conn)
        .unwrap();

    let sim_params = serde_json::from_value(sim_params_json).unwrap();

    let (step_val, t_val, latest_env_id) = env
        .filter(run_id.eq(rid as i32))
        .select((step, t, schema::env::id))
        .order(step.desc())
        .first::<(i32, f64, i32)>(conn)
        .unwrap();

    let agent_vals: Vec<(f64, f64, f64, f64)> = agent
        .filter(env_id.eq(latest_env_id))
        .select((rx, ry, ux, uy))
        .order(agent_id.asc())
        .get_results::<(f64, f64, f64, f64)>(conn)
        .unwrap();

    let rx_arr = Array1::from_iter(agent_vals.iter().map(|x| x.0));
    let ry_arr = Array1::from_iter(agent_vals.iter().map(|x| x.1));
    let ux_arr = Array1::from_iter(agent_vals.iter().map(|x| x.2));
    let uy_arr = Array1::from_iter(agent_vals.iter().map(|x| x.3));

    let r = ndarray::stack(Axis(1), &[rx_arr.view(), ry_arr.view()]).unwrap();
    let u_p = ndarray::stack(Axis(1), &[ux_arr.view(), uy_arr.view()]).unwrap();

    let sim_state = SimState {
        step: step_val as usize,
        t: t_val,
        r,
        u_p,
    };

    return (sim_params, sim_state);
}
