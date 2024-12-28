use nalgebra::Vector3;
use num_traits::Zero;

#[derive(Debug, Clone)]
pub struct Wrench {
    pub force: Vector3<f64>,
    pub torque: Vector3<f64>,
}

impl Zero for Wrench {
    fn zero() -> Self {
        Self {
            force: Vector3::zeros(),
            torque: Vector3::zeros(),
        }
    }

    fn is_zero(&self) -> bool {
        self.force.is_zero() && self.torque.is_zero()
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
