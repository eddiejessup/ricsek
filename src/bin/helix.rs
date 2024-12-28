use nalgebra::{Point3, UnitVector3, Vector3};

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

    // Position and unit tangent vector at position t.
    fn eval(&self, t: f64) -> (Point3<f64>, UnitVector3<f64>) {
        let pitch_norm = self.pitch_norm();

        let x = self.radius * t.cos();
        let y = self.radius * t.sin();
        let z = pitch_norm * t;
        let r = Point3::new(x, if self.left_handed { -y } else { y }, z);

        let tang = UnitVector3::new_unchecked(
            Vector3::new(-r.y, r.x, pitch_norm) / (self.radius.powi(2) + pitch_norm.powi(2)).sqrt(),
        );

        (r, tang)
    }

    fn eval_at_frac(&self, frac: f64) -> (Point3<f64>, UnitVector3<f64>) {
        let n_turns = self.length / self.pitch;
        let max_t = n_turns * std::f64::consts::TAU;
        let t = frac * max_t;
        self.eval(t)
    }

    pub fn eval_at_fracs(&self, n_points: usize) -> Vec<(Point3<f64>, UnitVector3<f64>)> {
        (0..=n_points)
            .map(|i| self.eval_at_frac(i as f64 / n_points as f64))
            .collect::<Vec<_>>()
    }
}

fn main() {
    env_logger::init();

    let n_points = 90;

    let helix = Helix {
        radius: 1.0,
        pitch: 1.0,
        length: 3.0,
        left_handed: false,
    };
    let helix_coords = helix.eval_at_fracs(n_points);
    // Print the coordinates nicely.
    for (i, (p, tang)) in helix_coords.iter().enumerate() {
        println!(
            "{}: pos={}, tangent={}, tangent_mag={}",
            i,
            p,
            tang.into_inner(),
            tang.magnitude()
        );
    }
}
