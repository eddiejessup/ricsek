use std::ops::Sub;

use ndarray::prelude::*;
use ndarray::Array2;
use ndarray_rand::rand_distr::Normal;
use ndarray_rand::RandomExt;
use ricsek::common::*;

fn wrap(d: &mut Array2<f64>, l: f64) {
    let l_half = l * 0.5;
    d.mapv_inplace(|mut x| {
        if x < -l_half {
            x += l
        } else if x > l_half {
            x -= l
        };
        x
    });
}

fn segment_repulsion_v_1(
    r: ArrayView1<f64>,
    u: ArrayView1<f64>,
    segment: &LineSegment,
    k: f64,
) -> Array1<f64> {
    // https://arxiv.org/pdf/0806.2898.pdf
    // The velocity component of the swimmer towards the segment:
    //   v_y(θ, y) = (−3p / 64πηy^2) * (1 − 3 cos^2(θ)).
    // Where:
    // - p is the dipole strength
    // - θ is the angle between the segment normal and the swimmer orientation.
    // - y is the distance between the swimmer and the segment.

    // The nearest point on the segment to the swimmer.
    // println!("r: {:?}", 1e6 * &r);
    // println!("u (orientation): {:?}", u);
    let r_seg = segment.nearest_point(r);
    // println!("r_seg: {:?}", 1e6 * &r_seg);

    // The vector from the segment to the swimmer.
    let y_vec = r.sub(r_seg);
    // println!("y_vec: {:?}", 1e6 * &y_vec);
    // The distance from the segment to the swimmer.
    let y = norm_one_vec(&y_vec);
    // println!("y: {:?}", 1e6 * &y);

    // To find the angle between the swimmer's direction and the segment normal:
    //   cos(θ) = y_vec_unit ⋅ u
    let y_vec_unit = y_vec / y;
    // println!("y_vec_unit: {:?}", y_vec_unit);
    let cos_th = y_vec_unit.dot(&u);
    // println!("cos_th: {:?}", cos_th);
    // println!("th: {:?} degrees", cos_th.acos().to_degrees());

    // The velocity.
    let v = -(k / y.powi(2)) * (1.0 - 3.0 * cos_th.powi(2));
    // println!("v: {:?}", 1e6 * &v);
    return v * &y_vec_unit;
}

fn segment_repulsion_v_n(
    r: ArrayView2<f64>,
    u: ArrayView2<f64>,
    segment: &LineSegment,
    k: f64,
) -> Array2<f64> {
    let mut f = Array::zeros((r.nrows(), 2));
    for i in 0..r.nrows() {
        f.row_mut(i)
            .assign(&segment_repulsion_v_1(r.row(i), u.row(i), segment, k));
    }
    f
}

pub fn run(
    conn: &mut diesel::PgConnection,
    sim_params: SimParams,
    segments: Vec<LineSegment>,
    mut sim_state: SimState,
    run_params: RunParams,
) {
    let trans_diff_distr = Normal::new(0.0, sim_params.len_trans_diff()).unwrap();
    let rot_diff_distr = Normal::new(0.0, sim_params.len_rot_diff()).unwrap();

    while sim_state.t < run_params.t_max {
        // Compute environment and agent variables.

        // Update agent position.
        let mut dr = Array::zeros((sim_params.n, 2));
        // Compute propulsion translation.
        // Overall agent velocity.
        let mut v = Array::zeros((sim_params.n, 2));
        // Agent propulsion.
        v.scaled_add(
            sim_params.ag_f_propulse * sim_params.ag_trans_mobility,
            &sim_state.u_p,
        );

        // Update position due to segment-agent repulsion.
        for segment in &segments {
            v += &segment_repulsion_v_n(
                sim_state.r.view(),
                sim_state.u_p.view(),
                &segment,
                sim_params.k_repulse,
            );
        }
        // Update agent position from velocity.
        dr.scaled_add(sim_params.dt, &v);
        // Compute translational diffusion translation.
        dr += &Array::random((sim_params.n, 2), trans_diff_distr);

        // Perform the translation.
        sim_state.r += &dr;
        // Apply periodic boundary condition.
        wrap(&mut (sim_state.r), sim_params.l);

        // Update agent direction.
        let mut dth = Array::zeros(sim_params.n);
        // Compute rotational diffusion rotation.
        dth += &Array::random((sim_params.n,), rot_diff_distr);

        // Compute torque rotation.
        let torque = Array::zeros(sim_params.n);
        dth.scaled_add(sim_params.dt * sim_params.ag_rot_mobility, &torque);

        // Perform the rotation.
        rotate_2d_vecs_inplace(&mut sim_state.u_p.view_mut(), dth.view());

        // (C). Update environment

        // Upate time and step.
        sim_state.t += sim_params.dt;
        sim_state.step += 1;

        if sim_state.step % run_params.dstep_view == 0 {
            println!("CHECKPOINT: step={}, t = {}", sim_state.step, sim_state.t);
            ricsek::db::write_checkpoint(conn, run_params.run_id, &sim_state);
        }
    }
}

fn main() {
    let dt_view = 0.08;
    let t_max = 40.0;

    let conn = &mut ricsek::db::establish_connection();

    let run_id = ricsek::db::read_latest_run_id(conn);
    println!("Running run_id: {}", run_id);
    let sim_setup = ricsek::db::read_run(conn, run_id);
    let sim_state = ricsek::db::read_latest_checkpoint(conn, run_id);
    println!("Running from step {}, t={}s", sim_state.step, sim_state.t);

    let run_params = RunParams {
        t_max,
        dstep_view: sim_setup.params.to_steps(dt_view),
        run_id,
    };

    run(
        conn,
        sim_setup.params,
        sim_setup.segments,
        sim_state,
        run_params,
    );
    println!("Done!");
}
