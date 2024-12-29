use crate::config::setup::parameters::common::BoundaryConfig;
use crate::config::setup::parameters::singularities::SingularityParams;
use crate::geometry::point::ObjectPoint;
use nalgebra::{zero, Vector3};

pub fn get_singularity_image_across_plane(
    d: Vector3<f64>,
    params: &SingularityParams,
) -> Vec<SingularityParams> {
    let h = d.norm();
    if h < 1e-12 {
        panic!("d is too small");
    }

    let d_direction = d.normalize();

    match params {
        SingularityParams::Stokeslet { a: stokeslet_force } => {
            let stokeslet_force_normal = (stokeslet_force.dot(&d) / h.powi(2)) * d;
            let stokeslet_force_tangent = stokeslet_force - stokeslet_force_normal;

            let stokeslet_strength_normal = stokeslet_force_normal.norm();
            let stokeslet_strength_tangent = stokeslet_force_tangent.norm();

            let stokeslet_direction_normal = if stokeslet_strength_normal > 0.0 {
                stokeslet_force_normal.normalize()
            } else {
                zero()
            };
            let stokeslet_direction_tangent = if stokeslet_strength_tangent > 0.0 {
                stokeslet_force_tangent.normalize()
            } else {
                zero()
            };

            let stokes_doublet_normal_mag = (2.0 * h * stokeslet_strength_normal).sqrt();
            let stokes_doublet_tangent_mag = (2.0 * h * stokeslet_strength_tangent).sqrt();
            let potential_doublet_mag = 2.0 * h.powi(2);

            // https://sci-hub.ee/10.1017/S0305004100049902
            vec![
                // Stokeslet
                SingularityParams::Stokeslet {
                    a: -stokeslet_force,
                },
                // Stokes doublet, normal
                SingularityParams::StokesDoublet {
                    a: -stokes_doublet_normal_mag * stokeslet_direction_normal,
                    b: -stokes_doublet_normal_mag * d_direction,
                },
                // Potential doublet, normal
                SingularityParams::PotentialDoublet {
                    d: potential_doublet_mag * stokeslet_force_normal,
                },
                // Stokes doublet, tangential
                SingularityParams::StokesDoublet {
                    a: -stokes_doublet_tangent_mag * stokeslet_direction_tangent,
                    b: stokes_doublet_tangent_mag * d_direction,
                },
                // Potential doublet, tangential
                SingularityParams::PotentialDoublet {
                    d: -potential_doublet_mag * stokeslet_force_tangent,
                },
            ]
        }
        SingularityParams::StokesDoublet { a: _, b: _ } => {
            panic!("StokesDoublet not supported")
        }
        SingularityParams::Rotlet { c: _ } => {
            panic!("Rotlet not supported")
        }
        SingularityParams::Stresslet { a: _, b: _ } => {
            panic!("Stresslet not supported")
        }
        SingularityParams::PotentialDoublet { d: _ } => {
            panic!("PotentialDoublet not supported")
        }
    }
}

pub fn get_singularity_image(
    _p: &ObjectPoint,
    _params: &SingularityParams,
    boundaries: &BoundaryConfig,
) -> Vec<(ObjectPoint, SingularityParams)> {
    let param_sets_per_axis: Vec<Vec<(ObjectPoint, SingularityParams)>> = boundaries
        .0
        .iter()
        .enumerate()
        .filter_map(|(_i_axis, axis)| {
            if axis.closed {
                panic!("Closed boundaries not currently supported");
                // Find the vector pointing to the nearest boundary along this axis.
                // let d = Vector3::ith(
                //     i_axis,
                //     if p.position_com[i_axis] > 0.0 {
                //         1.0
                //     } else {
                //         -1.0
                //     } * (axis.l_half() - p.position[i_axis]),
                // );
                // Some(
                //     get_singularity_image_across_plane(d, params)
                //         .iter()
                //         .map(|params| {
                //             (
                //                 // Place each singularity on the opposite side of the boundary.
                //                 ObjectPoint {
                //                     // ID to indicate fictional singularities.
                //                     object_id: std::u32::MAX - 1,
                //                     position_com: p.position_com + 2.0 * d,
                //                     position: p.position + 2.0 * d,
                //                 },
                //                 params.clone(),
                //             )
                //         })
                //         .collect(),
                // )
            } else {
                None
            }
        })
        .collect();
    if param_sets_per_axis.len() > 1 {
        panic!("More than one boundary is closed");
    }
    param_sets_per_axis.iter().flatten().cloned().collect()
}
