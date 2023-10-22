use approx::assert_relative_eq;
use nalgebra::{Point3, Vector3};
use ricsek::{
    config::setup::parameters::singularities::{
        singularities_fluid_v_multi, Singularity, SingularityParams,
    },
    cuda::LinearFieldsContext,
};

fn test_pairwise_and_forces() {
    // Define box dimensions
    let l = Vector3::new(100.0, 100.0, 100.0);

    // Define two positions
    let eval_points = vec![
        (0, Point3::new(0.0, 0.0, 0.0)),
        (1, Point3::new(1.0, 0.0, 0.0)),
    ];

    let singularities = vec![
        (
            0,
            Singularity {
                point: Point3::new(0.0, 0.0, 0.0),
                params: SingularityParams::Stokeslet {
                    a: Vector3::new(1.0, 0.0, 0.0),
                },
            },
        ),
        (
            1,
            Singularity {
                point: Point3::new(1.0, 0.0, 0.0),
                params: SingularityParams::Stokeslet {
                    a: Vector3::new(-1.0, 0.0, 0.0),
                },
            },
        ),
    ];

    // Create the LinearFieldsContext
    let context = LinearFieldsContext::new(eval_points.len(), singularities.len(), l);

    // Compute pairwise distances
    let eval_vs_gpu = context.evaluate(&eval_points, &singularities, 32);

    let eval_vs_cpu = singularities_fluid_v_multi(&eval_points, &singularities);

    assert!(
        eval_vs_gpu.len() == eval_vs_cpu.len(),
        "eval_vs_gpu.len() = {}",
        eval_vs_gpu.len()
    );

    println!("eval_vs_gpu = {:?}", eval_vs_gpu);
    println!("eval_vs_cpu = {:?}", eval_vs_cpu);

    // Forces should have same magnitude but opposite directions
    // Also, according to F=1/r^2, the force should have a magnitude of 1.0
    for (f1, f2) in eval_vs_gpu.iter().zip(eval_vs_cpu.iter()) {
        assert_relative_eq!(f1, f2, epsilon = 1e-6);
    }
}

fn main() {
    test_pairwise_and_forces();
}
