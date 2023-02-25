use diesel::prelude::*;
use ndarray::prelude::*;
use ndarray::Zip;
use ndarray_rand::rand_distr::Uniform;
use ndarray_rand::RandomExt;
use ricsek::runner::common::*;
use time;

fn initialize_run(conn: &mut PgConnection, sim_params: &SimParams) -> i32 {
    use diesel::dsl::Eq;
    use ricsek::schema::run::dsl::*;
    return diesel::insert_into(ricsek::schema::run::table)
        .values((
            None::<Eq<id, i32>>,
            None::<Eq<created_at, time::OffsetDateTime>>,
            params.eq(serde_json::to_value(&sim_params).unwrap()),
        ))
        .returning(id)
        .get_result(conn)
        .unwrap();
}

fn write_checkpoint(
    conn: &mut PgConnection,
    rid: i32,
    step_view_: i32,
    sim_state: &SimState,
) -> i32 {
    use diesel::dsl::Eq;
    use ricsek::schema::env::dsl::*;
    let eid: i32 = diesel::insert_into(env)
        .values((
            None::<Eq<id, i32>>,
            run_id.eq(rid),
            step_view.eq(step_view_),
            step_sim.eq(sim_state.step_sim as i32),
            t_sim.eq(sim_state.t_sim),
        ))
        .returning(id)
        .get_result(conn)
        .unwrap();

    use ricsek::schema::agent::dsl::*;
    let tups = Zip::indexed(sim_state.r.view().rows())
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

    // None::<Eq<id, i32>>,
    diesel::insert_into(agent)
        .values(tups)
        .execute(conn)
        .unwrap();
    return eid;
}

fn main() {
    let sim_params = SimParams {
        dt_sim: 0.1,
        l: 1.0,
        v_propulse: 0.1,
        segments: Array3::default((0, 2, 2)),
        segment_repulse_v_0: 1.0,
        segment_repulse_d_0: 0.01,
        segment_align_omega_0: 50.0,
        segment_align_d_0: 0.01,
        ag_repulse_d_0: 1.0,
        ag_repulse_v_0: 1.0,
        d_trans_diff: 0.00002,
        d_rot_diff: 0.1,
        n: 10,
    };

    let r = Array::random(
        (sim_params.n, 2),
        Uniform::new(-sim_params.l * 0.5, sim_params.l * 0.5),
    );

    let mut u_p = Array::<f64, Ix2>::zeros((sim_params.n, 2));
    u_p.slice_mut(s![.., 0]).fill(1.0);
    rotate_2d_vecs_inplace(
        &mut u_p.view_mut(),
        Array::random(
            sim_params.n,
            Uniform::new(-std::f64::consts::PI, std::f64::consts::PI),
        )
        .view(),
    );

    let sim_state = SimState::new(u_p, r);

    let connection = &mut ricsek::establish_connection();

    let run_id = initialize_run(connection, &sim_params);
    write_checkpoint(connection, run_id, 0, &sim_state);
}
