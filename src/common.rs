use std::f64::consts::PI;
use std::ops::Sub;

use ndarray::prelude::*;
use ndarray::Zip;

pub fn norm_sq_one_vec(v: &Array1<f64>) -> f64 {
    v.dot(v)
}

pub fn norm_one_vec(v: &Array1<f64>) -> f64 {
    norm_sq_one_vec(v).sqrt()
}

pub struct SimState {
    pub u_p: Array2<f64>,
    pub r: Array2<f64>,
    pub t: f64,
    pub step: usize,
}

impl SimState {
    pub fn new(u_p: Array2<f64>, r: Array2<f64>) -> SimState {
        SimState {
            u_p,
            r,
            t: 0.0,
            step: 0,
        }
    }
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct SimParams {
    pub dt: f64,
    pub l: f64,
    pub ag_f_propulse: f64,
    pub k_repulse: f64,
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

    pub fn len_rot_diff(&self) -> f64 {
        (2.0 * self.d_rot_diff * self.dt).sqrt()
    }

    pub fn len_trans_diff(&self) -> f64 {
        (2.0 * self.d_trans_diff * self.dt).sqrt()
    }

    pub fn l_half(&self) -> f64 {
        self.l * 0.5
    }

    pub fn force_to_velocity(&self, f: f64) -> f64 {
        self.ag_trans_mobility * f
    }
}

pub struct SimParamsPhysical {
    pub dt: f64,
    pub l: f64,
    // TODO: Derive `ag_f_propulse` and `ag_dipole_strength` from base
    // quantities.
    pub ag_f_propulse: f64,
    pub ag_dipole_strength: f64,
    pub temp: f64,
    pub ag_radius: f64,
    pub viscosity: f64,
    pub n: usize,
}

impl SimParamsPhysical {
    fn stokes_trans_coeff(&self) -> f64 {
        return 6.0 * PI * self.viscosity * self.ag_radius;
    }

    fn stokes_rot_coeff(&self) -> f64 {
        return 8.0 * PI * self.viscosity * self.ag_radius.powi(3);
    }

    fn stokes_trans_mobility(&self) -> f64 {
        // F = 6πηav
        return 1.0 / self.stokes_trans_coeff();
    }

    fn stokes_rot_mobility(&self) -> f64 {
        // T = 8πηa^3ω
        return 1.0 / self.stokes_rot_coeff();
    }

    fn thermal_energy(&self) -> f64 {
        return physical_constants::BOLTZMANN_CONSTANT * self.temp;
    }

    fn stokes_d_trans(&self) -> f64 {
        // D = kBT / 6πηa
        return self.thermal_energy() / self.stokes_trans_coeff();
    }

    fn stokes_d_rot(&self) -> f64 {
        // Drot = kBT / 8πηa^3
        return self.thermal_energy() / self.stokes_rot_coeff();
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

        return SimParams {
            dt: self.dt,
            l: self.l,
            n: self.n,
            ag_f_propulse: self.ag_f_propulse,
            k_repulse,
            ag_trans_mobility: self.stokes_trans_mobility(),
            ag_rot_mobility: self.stokes_rot_mobility(),
            d_trans_diff: self.stokes_d_trans(),
            d_rot_diff: self.stokes_d_rot(),
        };
    }
}

pub struct SimSetup {
    pub params: SimParams,
    pub segments: Vec<LineSegment>,
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub fn asarray(&self) -> Array1<f64> {
        return array![self.x, self.y];
    }
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct LineSegment {
    pub p1: Point,
    pub p2: Point,
}

impl LineSegment {
    pub fn centre(&self) -> Point {
        let arr = (self.p1.asarray() + self.p2.asarray()) * 0.5;
        Point {
            x: arr[0],
            y: arr[1],
        }
    }

    pub fn as_vector(&self) -> Array1<f64> {
        self.p2.asarray() - self.p1.asarray()
    }

    pub fn angle_to_x(&self) -> f64 {
        let v = self.as_vector();
        v[1].atan2(v[0])
    }

    pub fn length(&self) -> f64 {
        norm_one_vec(&self.as_vector())
    }

    pub fn nearest_point(&self, p: ArrayView1<f64>) -> Array1<f64> {
        let ss = self.p1.asarray();
        // Get vector from start to end of segment
        let dr = self.p2.asarray() - &ss;
        // Get vector from start of segment to point
        let ss_p = p.sub(&ss);
        // Get the component of ss_p along dr, which is basically the distance we
        // should go along the segment to get to the closest point.
        let nx_line = dr.dot(&ss_p) / norm_sq_one_vec(&dr);
        // Clamp the distance to the segment so we don't go past the end points of
        // the segment.
        let nx_seg = nx_line.min(1.0).max(0.0);
        // Return the point on the segment.
        ss + (dr * nx_seg)
    }
}

pub struct RunParams {
    pub t_max: f64,
    pub dstep_view: usize,
    pub run_id: usize,
}

pub fn rotate_2d_vec_inplace(v: &mut ArrayViewMut1<f64>, theta: f64) {
    let v0 = *v.get(0).unwrap();
    let v1 = *v.get(1).unwrap();
    *v.get_mut(0).unwrap() = v0 * theta.cos() - v1 * theta.sin();
    *v.get_mut(1).unwrap() = v0 * theta.sin() + v1 * theta.cos();
}

pub fn rotate_2d_vecs_inplace(v: &mut ArrayViewMut2<f64>, theta: ArrayView1<f64>) {
    Zip::from(v.rows_mut())
        .and(theta)
        .for_each(|mut x, theta| rotate_2d_vec_inplace(&mut x, *theta));
}
