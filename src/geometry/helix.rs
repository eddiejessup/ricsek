use log::debug;
use nalgebra::{Point3, Rotation3, UnitVector3, Vector3};

#[derive(serde::Serialize, serde::Deserialize, Clone, Debug)]
pub struct Helix {
    pub radius: f64,
    pub pitch: f64,
    pub length: f64,
    pub left_handed: bool,
}

impl Helix {
    fn pitch_norm(&self) -> f64 {
        self.pitch / std::f64::consts::TAU
    }

    // Position if helix were aligned with z-axis, starting at the origin
    fn r_standard(&self, s: f64, phase: f64) -> Point3<f64> {
        Point3::new(
            self.radius * (s + phase).cos(),
            self.radius * (s + phase).sin() * if self.left_handed { -1.0 } else { 1.0 },
            self.pitch_norm() * s,
        )
    }

    // Position if helix were aligned with z-axis, starting at the origin
    fn orient(&self, direction: UnitVector3<f64>) -> Rotation3<f64> {
        Rotation3::rotation_between(&Vector3::z(), &direction.into_inner())
            .unwrap_or_else(|| Rotation3::identity())
    }

    // Point on helix at offset length `s`, starting at `origin` in direction `direction`
    pub fn r(
        &self,
        origin: Point3<f64>,
        direction: UnitVector3<f64>,
        s: f64,
        phase: f64,
    ) -> Point3<f64> {
        debug!(
            "origin: {:?}, magnitude: {:?}",
            origin,
            origin.coords.magnitude()
        );
        debug!(
            "direction: {:?}, magnitude: {:?}",
            direction,
            direction.into_inner().magnitude()
        );
        debug!("s: {:?}", s);
        debug!("phase: {:?}", phase);
        let r_standard = self.r_standard(s, phase);
        debug!(
            "r_standard: {:?}, magnitude: {:?}",
            r_standard,
            r_standard.coords.magnitude()
        );
        let r_direction = self.orient(direction) * r_standard.coords;
        debug!(
            "r_direction: {:?}, magnitude: {:?}",
            r_direction,
            r_direction.magnitude()
        );
        let r = origin + r_direction;
        debug!("r: {:?}, magnitude: {:?}", r, r.coords.magnitude());
        debug!("--------------------------------");
        r
    }

    pub fn tangent_standard(&self, s: f64, phase: f64) -> UnitVector3<f64> {
        let pitch_norm = self.pitch_norm();
        let r = self.r_standard(s, phase);
        UnitVector3::new_unchecked(
            Vector3::new(-r.y, r.x, pitch_norm) / (self.radius.powi(2) + pitch_norm.powi(2)).sqrt(),
        )
    }

    pub fn tangent(&self, direction: UnitVector3<f64>, s: f64, phase: f64) -> UnitVector3<f64> {
        debug!("s: {:?}", s);
        debug!("phase: {:?}", phase);
        let t = self.tangent_standard(s, phase);
        let t_direction = UnitVector3::new_unchecked(self.orient(direction) * t.into_inner());
        debug!(
            "t_direction: {:?}, magnitude: {:?}",
            t_direction,
            t_direction.magnitude()
        );
        debug!("--------------------------------");
        t_direction
    }

    pub fn v(&self, direction: UnitVector3<f64>, s: f64, phase: f64, omega: f64) -> Vector3<f64> {
        let t = self.tangent_standard(s, phase);
        let v_std = -Vector3::new(t.x, t.y, 0.0).scale(self.radius * omega);
        self.orient(direction) * v_std
    }

    // Compute drag coefficients (RFT)
    pub fn drag_coeffs(&self, mu: f64) -> (f64, f64) {
        let log_term = (self.length / self.radius).ln();
        let zeta_par = (std::f64::consts::TAU * mu) / (log_term - 0.5);
        let zeta_perp = (2.0 * std::f64::consts::TAU * mu) / (log_term + 0.5);
        (zeta_par, zeta_perp)
    }
}
