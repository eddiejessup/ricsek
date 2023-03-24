use std::f64::consts::PI;

use super::simulation::SimParams;

pub struct PhysicalParams {
  pub dt: f64,
  pub l: f64,
  // TODO: Derive `ag_f_propulse` and `ag_dipole_strength` from base
  // quantities.
  pub ag_f_propulse: f64,
  pub ag_dipole_strength: f64,
  pub aspect_ratio: f64,
  pub temp: f64,
  pub ag_radius: f64,
  pub viscosity: f64,
  pub n: usize,
  pub seg_surface_stiffness: f64,
}

impl PhysicalParams {
  fn stokes_trans_coeff(&self) -> f64 {
      6.0 * PI * self.viscosity * self.ag_radius
  }

  fn stokes_rot_coeff(&self) -> f64 {
      8.0 * PI * self.viscosity * self.ag_radius.powi(3)
  }

  pub fn stokes_trans_mobility(&self) -> f64 {
      // F = 6πηav
      1.0 / self.stokes_trans_coeff()
  }

  pub fn stokes_rot_mobility(&self) -> f64 {
      // T = 8πηa^3ω
      1.0 / self.stokes_rot_coeff()
  }

  fn thermal_energy(&self) -> f64 {
      physical_constants::BOLTZMANN_CONSTANT * self.temp
  }

  fn stokes_d_trans(&self) -> f64 {
      // D = kBT / 6πηa
      self.thermal_energy() / self.stokes_trans_coeff()
  }

  fn stokes_d_rot(&self) -> f64 {
      // Drot = kBT / 8πηa^3
      self.thermal_energy() / self.stokes_rot_coeff()
  }

  pub fn force_to_velocity(&self, f: f64) -> f64 {
    self.stokes_trans_mobility() * f
  }

  pub fn f_hertz_coeff(&self) -> f64 {
    // Assume sphere for now.
    (4.0 / 3.0) * self.seg_surface_stiffness * self.ag_radius.sqrt()
  }

  pub fn v_hertz_coeff(&self) -> f64 {
    self.stokes_trans_coeff() * self.f_hertz_coeff()
  }

  pub fn as_params(&self) -> SimParams {
      // dipole strength per unit viscosity
      // I guess like how well the dipole transmits over distance.
      // N m / (Pa s)
      // If we divide by y^2:
      // N / (m Pa s)
      // N / (m (N/m^2) s)
      // 1 / (m (1/m^2) s)
      // m^2 / (m s)
      // m / s
      // speed! good.
      let k_repulse = 3.0 * self.ag_dipole_strength / (64.0 * PI * self.viscosity);
      SimParams {
          dt: self.dt,
          l: self.l,
          n: self.n,
          // Capsule electrostatics.
          ag_radius: self.ag_radius,
          seg_v_overlap_coeff: self.v_hertz_coeff(),
          // Propulsion.
          ag_v_propulse: self.force_to_velocity(self.ag_f_propulse),
          // Capsule hydrodynamics.
          k_repulse,
          aspect_ratio: self.aspect_ratio,

          // Diffusion.
          d_trans_diff: self.stokes_d_trans(),
          d_rot_diff: self.stokes_d_rot(),
      }
  }
}
