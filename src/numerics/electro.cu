#include <stdio.h>
#include "cuda_interface.h"
#include "cuda_common.h"

__constant__ BoundaryConfig d_bc;
__constant__ float d_radius;

__device__ V3Pair capsule_bounding_box(V3Pair a)
{
  return V3Pair{
      V3{
          fminf(a.a.x, a.b.x) - d_radius,
          fminf(a.a.y, a.b.y) - d_radius,
          fminf(a.a.z, a.b.z) - d_radius},
      V3{
          fmaxf(a.a.x, a.b.x) + d_radius,
          fmaxf(a.a.y, a.b.y) + d_radius,
          fmaxf(a.a.z, a.b.z) + d_radius}};
}

__device__ bool capsule_bbs_overlap(V3Pair a, V3Pair b)
{
  V3Pair a_bb = capsule_bounding_box(a);
  V3Pair b_bb = capsule_bounding_box(b);

  return (
      a_bb.a.x <= b_bb.b.x && a_bb.b.x >= b_bb.a.x && a_bb.a.y <= b_bb.b.y && a_bb.b.y >= b_bb.a.y && a_bb.a.z <= b_bb.b.z && a_bb.b.z >= b_bb.a.z);
}

__device__ V3 segment_centroid(V3Pair a)
{
  return V3{
      (a.a.x + a.b.x) / 2.0f,
      (a.a.y + a.b.y) / 2.0f,
      (a.a.z + a.b.z) / 2.0f};
}

__device__ V3 interpolate(V3 a, V3 b, float t)
{
  return V3{
      a.x + (b.x - a.x) * t,
      a.y + (b.y - a.y) * t,
      a.z + (b.z - a.z) * t};
}

__device__ V3Pair approx_closest_points_on_segments(V3Pair a, V3Pair b, unsigned int num_approx_points)
{
  // Generate 'n' equally spaced points from self.a to self.b.
  // Generate 'n' equally spaced points from p.a to p.b.
  // Compute pairwise distances.
  // Return the pair of points with the smallest distance.
  float min_dist_sq = 1e20; // Start with a large number
  V3Pair closest_pair;

  float step_size = 1.0f / (num_approx_points - 1);

  for (unsigned int i = 0; i < num_approx_points; ++i)
  {
    float t1 = (float)i * step_size;
    V3 point_on_a = interpolate(a.a, a.b, t1);

    for (unsigned int j = 0; j < num_approx_points; ++j)
    {
      float t2 = (float)j * step_size;
      V3 point_on_b = interpolate(b.a, b.b, t2);

      float dist_sq = length_sq(sub(point_on_a, point_on_b));

      if (dist_sq < min_dist_sq)
      {
        min_dist_sq = dist_sq;
        closest_pair = V3Pair{point_on_a, point_on_b};
      }
    }
  }
  return closest_pair;
}

__global__ void evaluateWrench(V3Pair *segments, V3Pair *pairwise_wrenches, unsigned int num_segments, unsigned int num_approx_points)
{
  unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
  unsigned int j = blockIdx.y * blockDim.y + threadIdx.y;

  if (i < num_segments && j < num_segments)
  {
    // printf("i=%u, j=%u\n", i, j);

    V3Pair segment_i = segments[i];
    V3Pair segment_j = segments[j];

    // Consider positions relative to segment_i, then apply periodic boundary conditions.
    V3 segment_i_centroid = segment_centroid(segment_i);
    segment_i = V3Pair{
        apply_boundary_conditions_point(sub(segment_i.a, segment_i_centroid), d_bc),
        apply_boundary_conditions_point(sub(segment_i.b, segment_i_centroid), d_bc),
    };
    segment_j = V3Pair{
        apply_boundary_conditions_point(sub(segment_j.a, segment_i_centroid), d_bc),
        apply_boundary_conditions_point(sub(segment_j.b, segment_i_centroid), d_bc),
    };

    unsigned int pair_idx = i * num_segments + j;
    if (i == j || !capsule_bbs_overlap(segment_i, segment_j))
    // if (i == j || false)
    {
      pairwise_wrenches[pair_idx] = zero_v3_pair();
      return;
    }

    V3Pair closest_segment_points = approx_closest_points_on_segments(segment_i, segment_j, num_approx_points);

    // In repulsion direction (from j to i)
    V3 r_seg_ji = sub(closest_segment_points.a, closest_segment_points.b);

    float overlap = 2.0 * d_radius - length(r_seg_ji);

    if (overlap <= 0.0f)
    {
      pairwise_wrenches[pair_idx] = zero_v3_pair();
      return;
    }

    V3 repulser_normal = unit(r_seg_ji);

    V3 repulser_force = scale(repulser_normal, sqrtf(overlap * overlap * overlap));

    // The force is applied at the closest-point on segment i, plus the radius
    // of the capsule in the direction of the repulsing object.
    // Note that we are working in the frame where the origin is the centroid of segment i,
    // ie where the moment should act.
    V3 moment_arm = add(closest_segment_points.a, scale(repulser_normal, -d_radius));
    V3 repulser_torque = cross(moment_arm, repulser_force);

    pairwise_wrenches[pair_idx] = V3Pair{repulser_force, repulser_torque};

    // printf("i=%u, j=%u, segment_i=(%f, %f, %f -> %f, %f, %f), closest point on i=(%f, %f, %f), overlap=%f, force=(%f, %f, %f), force_point=(%f, %f, %f), torque=(%f, %f, %f)\n",
    //        i, j,
    //        segment_i.a.x, segment_i.a.y, segment_i.a.z,
    //        segment_i.b.x, segment_i.b.y, segment_i.b.z,
    //        closest_segment_points.a.x, closest_segment_points.a.y, closest_segment_points.a.z,
    //        overlap,
    //        repulser_force.x, repulser_force.y, repulser_force.z,
    //        moment_arm.x, moment_arm.y, moment_arm.z,
    //        repulser_torque.x, repulser_torque.y, repulser_torque.z);
  }
}

__global__ void gatherWrenches(V3Pair *pairwise_wrenches, V3Pair *net_wrenches, unsigned int num_segments)
{
  unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;

  if (i < num_segments)
  {
    V3Pair wrench_tot = zero_v3_pair();

    for (unsigned int j = 0; j < num_segments; j++)
    {
      unsigned int pair_idx = i * num_segments + j;

      V3Pair w = pairwise_wrenches[pair_idx];

      wrench_tot.a.x += w.a.x;
      wrench_tot.a.y += w.a.y;
      wrench_tot.a.z += w.a.z;

      wrench_tot.b.x += w.b.x;
      wrench_tot.b.y += w.b.y;
      wrench_tot.b.z += w.b.z;
    }
    net_wrenches[i] = wrench_tot;
  }
}

extern "C"
{

  ElectroDeviceData *electro_init(unsigned int num_segments, BoundaryConfig bc, float radius)
  {
    ElectroDeviceData *device_data = (ElectroDeviceData *)malloc(sizeof(ElectroDeviceData));

    cudaMemcpyToSymbol(d_radius, &radius, sizeof(float));
    cudaMemcpyToSymbol(d_bc, &bc, sizeof(BoundaryConfig));

    unsigned int segments_size = num_segments * sizeof(V3Pair);
    V3Pair *d_segments;
    cudaMalloc((void **)&d_segments, segments_size);
    device_data->segments = d_segments;
    device_data->num_segments = num_segments;
    device_data->segments_size = segments_size;

    unsigned int pairwise_wrenches_size = num_segments * num_segments * sizeof(V3Pair);
    V3Pair *d_pairwise_wrenches;
    cudaMalloc((void **)&d_pairwise_wrenches, pairwise_wrenches_size);
    device_data->pairwise_wrenches = d_pairwise_wrenches;
    device_data->pairwise_wrenches_size = pairwise_wrenches_size;

    unsigned int net_wrenches_size = num_segments * sizeof(V3Pair);
    V3Pair *d_net_wrenches;
    cudaMalloc((void **)&d_net_wrenches, net_wrenches_size);
    device_data->net_wrenches = d_net_wrenches;
    device_data->net_wrenches_size = net_wrenches_size;
    return device_data;
  }

  void electro_evaluate(ElectroDeviceData *device_data, V3Pair *segments, V3Pair *net_wrenches, unsigned int threads_per_block_axis, unsigned int num_approx_points)
  {
    cudaMemcpy(device_data->segments, segments, device_data->segments_size, cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();

    unsigned int num_blocks_per_axis = (device_data->num_segments + threads_per_block_axis - 1) / threads_per_block_axis;
    dim3 block_structure = dim3(threads_per_block_axis, threads_per_block_axis);
    dim3 grid_structure = dim3(
        num_blocks_per_axis,
        num_blocks_per_axis);
    // printf("Grid shape: %d, %d\n", grid_structure.x, grid_structure.y);
    // printf("Block shape: %d, %d\n", block_structure.x, block_structure.y);
    evaluateWrench<<<grid_structure, block_structure>>>(
        device_data->segments,
        device_data->pairwise_wrenches,
        device_data->num_segments,
        num_approx_points);
    cudaDeviceSynchronize();
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess)
    {
      printf("Kernel '%s' launch failed with error: %s\n", "evaluateWrench", cudaGetErrorString(err));
    }

    dim3 gather_block_structure = dim3(threads_per_block_axis);
    dim3 gather_grid_structure = dim3(num_blocks_per_axis);
    gatherWrenches<<<gather_grid_structure, gather_block_structure>>>(device_data->pairwise_wrenches, device_data->net_wrenches, device_data->num_segments);
    cudaDeviceSynchronize();
    err = cudaGetLastError();
    if (err != cudaSuccess)
    {
      printf("Kernel '%s' launch failed with error: %s\n", "gatherWrenches", cudaGetErrorString(err));
    }

    cudaMemcpy(net_wrenches, device_data->net_wrenches, device_data->net_wrenches_size, cudaMemcpyDeviceToHost);
    cudaDeviceSynchronize();
  }

  void electro_finalize(ElectroDeviceData *device_data)
  {
    cudaFree(device_data->segments);
    cudaFree(device_data->pairwise_wrenches);
    cudaFree(device_data->net_wrenches);
    free(device_data);
  }
}
