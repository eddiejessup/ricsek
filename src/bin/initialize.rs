use nalgebra::{point, Unit, UnitComplex, Vector2};
use rand::distributions::Uniform;
use rand_distr::Distribution;
use ricsek::math::capsule::Capsule;
use ricsek::math::point::random_point;
use ricsek::parameters::physical::PhysicalParams;
use ricsek::state::*;

fn main() {
    let sim_params_phys = PhysicalParams {
        dt: 0.001,
        l: 200e-6,
        viscosity: 0.001,
        ag_f_propulse: 0.5e-12,
        ag_dipole_strength: 0.5e-18,
        aspect_ratio: 2.0,
        ag_radius: 2e-6,
        ag_area_density: 0.01e12,
        temp: 300.0,
        seg_surface_stiffness: 1e15,
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
    Mean area density: {ag_density:.1} cells/µm^2
    Propulsive force: {ag_f_propulse} pN
    Dipole strength: {ag_dipole_strength} pN·µm
    Effective radius: {ag_radius} µm

Derived parameters:
  Agents:
    Count: {ag_n}
    Translational mobility: {ag_trans_mobility:.1} (µm/s)/pN
    Translational diffusion rate: {d_trans_diff:.1} µm^2/s
    Rotational diffusion rate: {d_rot_diff:.1} rad^2/s

Computed derived parameters (for info only):
  Agents:
    Translational propulsive velocity: {ag_v_propulse:.1} µm/s
    Rotational diffusion randomisation timescale: {t_rot_diff:.1} s
    System crossing time: {t_cross:.1} s",
        dt = sim_params_phys.dt,
        temp = sim_params_phys.temp,
        l = 1e6 * sim_params_phys.l,
        viscosity = 1e3 * sim_params_phys.viscosity,
        ag_f_propulse = 1e12 * sim_params_phys.ag_f_propulse,
        ag_dipole_strength = 1e18 * sim_params_phys.ag_dipole_strength,
        ag_radius = 1e6 * sim_params_phys.ag_radius,
        ag_density = 1e-12 * sim_params_phys.ag_area_density,
        ag_trans_mobility = 1e-6 * sim_params_phys.stokes_trans_mobility(),
        ag_n = sim_params.n,
        d_trans_diff = 1e12 * sim_params.d_trans_diff,
        d_rot_diff = sim_params.d_rot_diff,
        ag_v_propulse = 1e6 * sim_params.ag_v_propulse,
        t_rot_diff = std::f64::consts::PI * std::f64::consts::PI / (4.0 * sim_params.d_rot_diff),
        t_cross = sim_params.l / sim_params.ag_v_propulse,
    );

    let mut rng = rand::thread_rng();
    let r_distr = Uniform::new(-sim_params.l * 0.5, sim_params.l * 0.5);
    let th_distr = Uniform::new(-std::f64::consts::PI, std::f64::consts::PI);
    let agents = (0..sim_params.n)
        .map(|_i| Agent {
            r: random_point(&mut rng, r_distr).into(),
            u: Unit::new_normalize(UnitComplex::new(th_distr.sample(&mut rng)) * Vector2::x()),
        })
        .collect();

    let capsule_radius = 20e-6;
    let capsules = vec![(point![0.0, -0.25], point![0.0, 0.25])]
        .iter()
        .map(|(s, e)| (s * sim_params_phys.l / 2.0, e * sim_params_phys.l / 2.0))
        .map(|(s, e)| Capsule::new(s, e, capsule_radius))
        .collect();

    let sim_state = SimState::new(agents);

    let connection = &mut ricsek::db::establish_connection();

    let run_id = ricsek::db::initialize_run(connection, &sim_params, &capsules);
    ricsek::db::write_checkpoint(connection, run_id, &sim_state);
    println!("Initialized run ID {}", run_id);
}
