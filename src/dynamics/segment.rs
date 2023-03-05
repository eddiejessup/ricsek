use geo::euclidean_distance::EuclideanDistance;

use crate::common::Agent;

fn agent_segment_kinematics(
    agent: &Agent,
    segment: &geo::Line,
    k: f64,
    aspect_ratio: f64,
) -> (geo::Point, f64) {
    // https://arxiv.org/pdf/0806.2898.pdf
    // The velocity component of the swimmer towards the segment:
    //   v_y(θ, y) = (−3p / 64πηy^2) * (1 − 3 cos^2(θ)).
    // Where:
    // - p is the dipole strength
    // - θ is the angle between the segment normal and the swimmer orientation.
    // - y is the distance between the swimmer and the segment.

    // The nearest point on the segment to the swimmer.
    let r_seg = match geo::ClosestPoint::closest_point(&segment, &agent.r) {
        geo::Closest::Intersection(_p) => {
            panic!("Intersection: of segment with point");
        }
        geo::Closest::Indeterminate => {
            panic!("No single closest point on segment to point");
        }
        geo::Closest::SinglePoint(p) => p,
    };

    // The vector from the segment to the swimmer.
    let y_vec = agent.r - r_seg;
    // The distance from the segment to the swimmer.
    let y = y_vec.euclidean_distance(&geo::Point::new(0.0, 0.0));

    // To find the angle between the swimmer's direction and the segment normal:
    //   cos(θ) = y_vec_unit ⋅ u
    let y_vec_unit = y_vec / y;
    let cos_th = y_vec_unit.dot(agent.u);

    // The velocity.
    let v = -(k / y.powi(2)) * (1.0 - 3.0 * cos_th.powi(2));

    let ar_factor = (aspect_ratio.powi(2) - 1.0) / (2.0 * (aspect_ratio.powi(2) + 1.0));
    let sin_th = (1.0 - cos_th.powi(2)).sqrt();
    let om = (-k * cos_th * sin_th / y.powi(3)) * (1.0 + ar_factor * (1.0 + cos_th.powi(2)));

    (y_vec_unit * v, om)
}

pub fn agent_segments_kinematics(
    agent: &Agent,
    segments: &[geo::Line],
    k: f64,
    aspect_ratio: f64,
) -> (geo::Point, f64) {
    segments
        .iter()
        .map(|segment| agent_segment_kinematics(agent, segment, k, aspect_ratio))
        .fold((geo::Point::new(0.0, 0.0), 0.0), |(v, om), (v1, om1)| {
            (v + v1, om + om1)
        })
}
