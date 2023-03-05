use std::f64::consts::PI;

pub struct SimParamsPhysical {
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
}

impl SimParamsPhysical {
    fn stokes_trans_coeff(&self) -> f64 {
        6.0 * PI * self.viscosity * self.ag_radius
    }

    fn stokes_rot_coeff(&self) -> f64 {
        8.0 * PI * self.viscosity * self.ag_radius.powi(3)
    }

    fn stokes_trans_mobility(&self) -> f64 {
        // F = 6πηav
        1.0 / self.stokes_trans_coeff()
    }

    fn stokes_rot_mobility(&self) -> f64 {
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
            ag_f_propulse: self.ag_f_propulse,
            k_repulse,
            aspect_ratio: self.aspect_ratio,
            ag_trans_mobility: self.stokes_trans_mobility(),
            ag_rot_mobility: self.stokes_rot_mobility(),
            d_trans_diff: self.stokes_d_trans(),
            d_rot_diff: self.stokes_d_rot(),
        }
    }
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct SimParams {
    pub dt: f64,
    pub l: f64,
    pub ag_f_propulse: f64,
    pub k_repulse: f64,
    pub aspect_ratio: f64,
    pub ag_trans_mobility: f64,
    pub ag_rot_mobility: f64,
    pub d_trans_diff: f64,
    pub d_rot_diff: f64,
    pub n: usize,
}

impl SimParams {
    pub fn to_steps(&self, t: f64) -> usize {
        (t / self.dt).ceil() as usize
    }

    pub fn l_half(&self) -> f64 {
        self.l * 0.5
    }

    pub fn force_to_velocity(&self, f: f64) -> f64 {
        self.ag_trans_mobility * f
    }
}

pub struct SimSetup {
    pub params: SimParams,
    pub segments: Vec<geo::Line>,
}

pub struct RunParams {
    pub t_max: f64,
    pub dstep_view: usize,
    pub run_id: usize,
}
