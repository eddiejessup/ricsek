use nalgebra::Vector3;

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone, PartialEq)]
pub struct AxisBoundaryConfig {
    pub l: f64,
    pub closed: bool,
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
}
