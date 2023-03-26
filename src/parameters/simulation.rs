#[derive(serde::Serialize, serde::Deserialize)]
pub struct SimParams {
    pub dt: f64,
    pub l: f64,
    pub hydro_k_repulse: f64,
    pub ag_radius: f64,
    pub aspect_ratio: f64,
    pub ag_v_propulse: f64,
    pub d_trans_diff: f64,
    pub d_rot_diff: f64,
    pub hertz_coeff: f64,
    pub n: usize,
}

impl SimParams {
    pub fn to_steps(&self, t: f64) -> usize {
        (t / self.dt).ceil() as usize
    }

    pub fn l_half(&self) -> f64 {
        self.l * 0.5
    }
}
