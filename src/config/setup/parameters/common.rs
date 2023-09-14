use nalgebra::{Point3, UnitVector3, Vector3};

use crate::{geometry::closest::Closest, state::Agent};

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone, PartialEq)]
pub struct AxisBoundaryConfig {
    pub l: f64,
    pub closed: bool,
}

impl AxisBoundaryConfig {
    pub fn l_half(&self) -> f64 {
        self.l * 0.5
    }
}

// Newtype around Vector3<AxisBoundaryConfig> for convenience functions.
#[derive(serde::Serialize, serde::Deserialize, Debug, Clone, PartialEq)]
pub struct BoundaryConfig(pub Vector3<AxisBoundaryConfig>);

impl BoundaryConfig {
    pub fn l(&self) -> Vector3<f64> {
        self.0.map(|b| b.l)
    }

    pub fn l_half(&self) -> Vector3<f64> {
        self.l() * 0.5
    }

    fn closest_point_on_boundary(&self, r: Point3<f64>, i: usize, positive: bool) -> Point3<f64> {
        let mut r_b = r;
        r_b[i] = self.0[i].l_half() * if positive { 1.0 } else { -1.0 };
        r_b
    }

    fn agent_closest_points_on_boundary(
        &self,
        a: &Agent,
        i: usize,
        positive: bool,
    ) -> (Point3<f64>, Point3<f64>) {
        let normal = UnitVector3::new_unchecked(Vector3::ith(i, if positive { -1.0 } else { 1.0 }));
        let subject_point = a.seg.furthest_point_along_axis(i, positive);
        let boundary_point = self.closest_point_on_boundary(subject_point, i, positive);
        (subject_point, boundary_point)
    }

    pub fn agent_closest_points_on_boundaries(&self, a: &Agent) -> Vec<(Point3<f64>, Point3<f64>)> {
        self.0
            .iter()
            .enumerate()
            .flat_map(|(i, b)| {
                if b.closed {
                    vec![
                        self.agent_closest_points_on_boundary(a, i, true),
                        self.agent_closest_points_on_boundary(a, i, false),
                    ]
                } else {
                    vec![]
                }
            })
            .collect()
    }
}
