use rand::distributions::Uniform;
use rand_distr::Distribution;
use ricsek::common::params::SimParamsPhysical;
use ricsek::common::*;
use ricsek::math::{rotate_point_inplace, random_point};

const PLANE_SEGMENTS: [geo::Line; 1] = [geo::Line {
    start: geo::coord! {x: 0.0, y: -50e-6},
    end: geo::coord! {x: 0.0, y: 50e-6},
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

    let mut rng = rand::thread_rng();
    let r_distr = Uniform::new(-sim_params.l * 0.5, sim_params.l * 0.5);
    let th_distr = Uniform::new(-std::f64::consts::PI, std::f64::consts::PI);
    let agents = (0..sim_params.n)
        .map(|_i| {
            let th = th_distr.sample(&mut rng);
            let mut u = [0.0, 1.0].into();
            rotate_point_inplace(&mut u, th);
            Agent {
                r: random_point(&mut rng, r_distr),
                u,
            }
        })
        .collect();

    let sim_state = SimState::new(agents);

    let connection = &mut ricsek::db::establish_connection();

    let run_id = ricsek::db::initialize_run(connection, &sim_params, &segments);
    ricsek::db::write_checkpoint(connection, run_id, &sim_state);
    println!("Initialized run ID {}", run_id);
}
