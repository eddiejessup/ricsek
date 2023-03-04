use ndarray::prelude::*;
use ndarray_rand::rand_distr::Uniform;
use ndarray_rand::RandomExt;
use ricsek::common::*;

const PLANE_SEGMENTS: [LineSegment; 1] = [LineSegment {
    p1: Point { x: 0.0, y: -50e-6 },
    p2: Point { x: 0.0, y: 50e-6 },
}];

fn main() {
    let segments = Vec::from(PLANE_SEGMENTS);
    // let segments = vec![];

    let sim_params_phys = SimParamsPhysical {
        dt: 0.01,
        l: 200e-6,
        viscosity: 0.001,
        ag_f_propulse: 0.5e-12,
        ag_dipole_strength: 0.5e-18,
        aspect_ratio: 2.0,
        ag_radius: 1e-6,
        n: 100,
        // temp: 300.0,
        temp: 0.0,
    };
    let sim_params = sim_params_phys.as_params();
    println!(
        "\
Physical parameters:
  Environment:
    Timestep: {dt} s
    Temperature: {temp} K
    System length: {l} µm
    Viscosity: {viscosity} mPa·s

  Agents:
    Propulsive force: {ag_f_propulse} pN
    Dipole strength: {ag_dipole_strength} pN·µm
    Effective radius: {ag_radius} µm
    Count: {n}

Derived parameters:
  Agents:
    Translational mobility: {ag_trans_mobility:.1} (µm/s)/pN
    Rotational mobility: {ag_rot_mobility:.1} (rad/s)/pN
    Translational diffusion rate: {d_trans_diff:.1} µm^2/s
    Rotational diffusion rate: {d_rot_diff:.1} rad^2/s

Computed derived parameters (for info only):
  Agents:
    Translational propulsive velocity: {ag_v_propulse:.1} µm/s
    Rotational diffusion randomisation timescale: {t_rot_diff:.1} s
    System crossing time: {l_cross:.1} s",
        dt = sim_params_phys.dt,
        temp = sim_params_phys.temp,
        l = 1e6 * sim_params_phys.l,
        viscosity = 1e3 * sim_params_phys.viscosity,
        ag_f_propulse = 1e12 * sim_params_phys.ag_f_propulse,
        ag_dipole_strength = 1e18 * sim_params_phys.ag_dipole_strength,
        ag_radius = 1e6 * sim_params_phys.ag_radius,
        n = sim_params_phys.n,
        ag_trans_mobility = 1e-6 * sim_params.ag_trans_mobility,
        ag_rot_mobility = 1e-12 * sim_params.ag_rot_mobility,
        d_trans_diff = 1e12 * sim_params.d_trans_diff,
        d_rot_diff = sim_params.d_rot_diff,
        ag_v_propulse = 1e6 * sim_params.force_to_velocity(sim_params.ag_f_propulse),
        t_rot_diff = std::f64::consts::PI * std::f64::consts::PI / (4.0 * sim_params.d_rot_diff),
        l_cross = sim_params.l / sim_params.force_to_velocity(sim_params.ag_f_propulse),
    );

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

    let connection = &mut ricsek::db::establish_connection();

    let run_id = ricsek::db::initialize_run(connection, &sim_params, &segments);
    ricsek::db::write_checkpoint(connection, run_id, &sim_state);
    println!("Initialized run ID {}", run_id);
}
