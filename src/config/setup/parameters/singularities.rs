use crate::dynamics::stokes_solutions::*;

use nalgebra::{zero, Point3, Vector3};

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
#[serde(tag = "type")]
pub enum SingularityParams {
    Stokeslet { a: Vector3<f64> },
    StokesDoublet { a: Vector3<f64>, b: Vector3<f64> },
    Rotlet { c: Vector3<f64> },
    Stresslet { a: Vector3<f64>, b: Vector3<f64> },
    PotentialDoublet { d: Vector3<f64> },
}

impl SingularityParams {
    pub fn eval(&self, r: Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
        match self {
            SingularityParams::Stokeslet { a } => (stokeslet_u(*a, r), stokeslet_vort(*a, r)),
            SingularityParams::StokesDoublet { a, b } => (
                stokes_doublet_chwang_u(*a, *b, r),
                stokes_doublet_chwang_vort(*a, *b, r),
            ),
            SingularityParams::Rotlet { c } => (rotlet_chwang_u(*c, r), rotlet_chwang_vort(*c, r)),
            SingularityParams::Stresslet { a, b } => (
                stresslet_chwang_u(*a, *b, r),
                stresslet_chwang_vort(*a, *b, r),
            ),
            SingularityParams::PotentialDoublet { d } => (
                potential_doublet_chwang_u(*d, r),
                potential_doublet_chwang_vort(*d, r),
            ),
        }
    }
}

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
pub struct Singularity {
    pub point: Point3<f64>,
    pub params: SingularityParams,
}

impl Singularity {
    pub fn eval(&self, r: Point3<f64>) -> (Vector3<f64>, Vector3<f64>) {
        self.params.eval(r - self.point)
    }
}

pub fn singularities_fluid_v(
    r_eval: Point3<f64>,
    i_eval: u32,
    singularities: &[(u32, Singularity)],
) -> (Vector3<f64>, Vector3<f64>) {
    singularities
        .iter()
        .fold((zero(), zero()), |(v_tot, vort_tot), (i_sing, sing)| {
            if i_eval == *i_sing {
                (v_tot, vort_tot)
            } else {
                let (v, vort) = sing.eval(r_eval);
                (v_tot + v, vort_tot + vort)
            }
        })
}

pub fn singularities_fluid_v_multi(
    eval_points: &[(u32, Point3<f64>)],
    singularities: &[(u32, Singularity)],
) -> Vec<(Vector3<f64>, Vector3<f64>)> {
    eval_points
        .iter()
        .map(|(i_eval, r_eval)| singularities_fluid_v(*r_eval, *i_eval, singularities))
        .collect()
}
