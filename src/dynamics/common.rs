use nalgebra::Vector3;

#[derive(Debug, Clone)]
pub struct Wrench {
    pub force: Vector3<f64>,
    pub torque: Vector3<f64>,
}

pub fn zero_wrench() -> Wrench {
    Wrench {
        force: Vector3::zeros(),
        torque: Vector3::zeros(),
    }
}

// Implement addition on wrenches
impl std::ops::Add for Wrench {
    type Output = Wrench;

    fn add(self, other: Wrench) -> Wrench {
        Wrench {
            force: self.force + other.force,
            torque: self.torque + other.torque,
        }
    }
}
