use approx::assert_relative_eq;
use nalgebra::{Point3, Vector3};
use ricsek::cuda::LinearFieldsContext;

fn test_pairwise_and_forces() {
    // Define box dimensions
    let l = Vector3::new(100.0, 100.0, 100.0);

    // Create the LinearFieldsContext
    let context = LinearFieldsContext::new(2, l);

    // Define two positions
    let mut positions = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)];

    // Compute pairwise distances
    context.populate_pairwise_distances(&mut positions, 32);

    let distances = context.fetch_pairwise_distances();

    assert!(distances.len() == 2 * 2, "distances.len() = {}", distances.len());

    println!("distances = {:?}", distances);

    // Compute net forces
    let forces = context.net_forces(32);

    println!("forces = {:?}", forces);

    // Forces should have same magnitude but opposite directions
    // Also, according to F=1/r^2, the force should have a magnitude of 1.0
    assert_relative_eq!(forces[0].magnitude(), 1.0, epsilon = 1e-6);
    assert_relative_eq!(forces[1].magnitude(), 1.0, epsilon = 1e-6);

    assert_relative_eq!(forces[0], -forces[1], epsilon = 1e-6);
}

fn main() {
    test_pairwise_and_forces();
}
