#include <stdio.h>
#include <cuda_ricsek.h>
#include <common.cu>

__constant__ BoundaryConfig d_bc;

__global__ void evaluateStokeslet(ObjectPoint *eval_points, Stokeslet *stokeslets, V3Pair *pairwise_twists, unsigned int num_eval_points, unsigned int num_stokeslets)
{
  unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
  unsigned int j = blockIdx.y * blockDim.y + threadIdx.y;

  if (i < num_eval_points && j < num_stokeslets)
  {
    ObjectPoint eval_point_i = eval_points[i];
    Stokeslet stokeslet_j = stokeslets[j];

    unsigned int pair_idx = i * num_eval_points + j;
    if (eval_point_i.object_id == stokeslet_j.object_point.object_id)
    {
      pairwise_twists[pair_idx] = zero_v3_pair();
      return;
    }

    // r is the vector pointing from the singularity to the point of interest.
    V3 r = apply_boundary_conditions(sub(eval_point_i.position, stokeslet_j.object_point.position), d_bc);

    float r1 = length(r);
    float r1_inv = 1.0 / r1;
    float r3_inv = 1.0 / (r1 * r1 * r1);

    // (a / |r|) + (a.dot(r) * r / |r|^3)
    float r_coeff = dot(stokeslet_j.force, r) * r3_inv;
    pairwise_twists[pair_idx].a = V3{
        (r1_inv * stokeslet_j.force.x) + (r_coeff * r.x),
        (r1_inv * stokeslet_j.force.y) + (r_coeff * r.y),
        (r1_inv * stokeslet_j.force.z) + (r_coeff * r.z)};
  }
}

__global__ void gatherSingularities(V3Pair *pairwise_twists, V3Pair *eval_point_twists, unsigned int num_eval_points, unsigned int num_stokeslets)
{
  unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;

  if (i < num_eval_points)
  {
    V3 v_tot = zero_v3();
    V3 vort_tot = zero_v3();

    for (unsigned int j = 0; j < num_stokeslets; j++)
    {
      unsigned int pair_idx = i * num_eval_points + j;

      V3 v = pairwise_twists[pair_idx].a;
      // printf("GATHER: i=%u, j=%u, v=(%f, %f, %f)\n", i, j, pairwise_twists[pair_idx].x, pairwise_twists[pair_idx].y, pairwise_twists[pair_idx].z);
      v_tot.x += v.x;
      v_tot.y += v.y;
      v_tot.z += v.z;

      V3 vort = pairwise_twists[pair_idx].b;
      vort_tot.x += vort.x;
      vort_tot.y += vort.y;
      vort_tot.z += vort.z;
    }
    eval_point_twists[i] = V3Pair {v_tot, vort_tot};

    // printf("GATHER-FINAL: i=%u, v_tot=(%f, %f, %f), eval_point_twists[i]=(%f, %f, %f)\n", i, v_tot.x, v_tot.y, v_tot.z, eval_point_twists[i].x, eval_point_twists[i].y, eval_point_twists[i].z);
  }
}

extern "C"
{

  FluidDeviceData *fluid_init(unsigned int num_eval_points, unsigned int num_stokeslets, BoundaryConfig bc)
  {
    FluidDeviceData *device_data = (FluidDeviceData *)malloc(sizeof(FluidDeviceData));

    cudaMemcpyToSymbol(d_bc, &bc, sizeof(BoundaryConfig));

    unsigned int eval_points_size = num_eval_points * sizeof(ObjectPoint);
    ObjectPoint *d_eval_points;
    cudaMalloc((void **)&d_eval_points, eval_points_size);
    device_data->eval_points = d_eval_points;
    device_data->eval_points_size = eval_points_size;
    device_data->num_eval_points = num_eval_points;

    unsigned int stokeslets_size = num_stokeslets * sizeof(Stokeslet);
    Stokeslet *d_stokeslets;
    cudaMalloc((void **)&d_stokeslets, stokeslets_size);
    device_data->stokeslets = d_stokeslets;
    device_data->stokeslets_size = stokeslets_size;
    device_data->num_stokeslets = num_stokeslets;

    unsigned int pairwise_twists_size = num_eval_points * num_stokeslets * sizeof(V3Pair);
    V3Pair *d_pairwise_twists;
    cudaMalloc((void **)&d_pairwise_twists, pairwise_twists_size);
    device_data->pairwise_twists = d_pairwise_twists;
    device_data->pairwise_twists_size = pairwise_twists_size;

    unsigned int eval_point_twists_size = num_eval_points * sizeof(V3Pair);
    V3Pair *d_eval_point_twists;
    cudaMalloc((void **)&d_eval_point_twists, eval_point_twists_size);
    device_data->eval_point_twists = d_eval_point_twists;
    device_data->eval_point_twists_size = eval_point_twists_size;
    return device_data;
  }

  void fluid_evaluate(FluidDeviceData *device_data, ObjectPoint *eval_points, Stokeslet *stokeslets, V3Pair *eval_point_twists, unsigned int threads_per_block_axis)
  {
    unsigned int num_blocks_x = (device_data->num_eval_points + threads_per_block_axis - 1) / threads_per_block_axis;
    unsigned int num_blocks_y = (device_data->num_stokeslets + threads_per_block_axis - 1) / threads_per_block_axis;

    dim3 block_structure = dim3(threads_per_block_axis, threads_per_block_axis);
    dim3 grid_structure = dim3(
        num_blocks_x,
        num_blocks_y);
    // printf("Grid shape: %d, %d\n", grid_structure.x, grid_structure.y);
    // printf("Block shape: %d, %d\n", block_structure.x, block_structure.y);
    cudaMemcpy(device_data->eval_points, eval_points, device_data->eval_points_size, cudaMemcpyHostToDevice);
    cudaMemcpy(device_data->stokeslets, stokeslets, device_data->stokeslets_size, cudaMemcpyHostToDevice);
    evaluateStokeslet<<<grid_structure, block_structure>>>(
        device_data->eval_points, device_data->stokeslets,
        device_data->pairwise_twists,
        device_data->num_eval_points, device_data->num_stokeslets);

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess)
    {
      printf("Kernel launch failed with error: %s\n", cudaGetErrorString(err));
    }

    dim3 gather_block_structure = dim3(threads_per_block_axis);
    dim3 gather_grid_structure = dim3(num_blocks_x);
    gatherSingularities<<<gather_grid_structure, gather_block_structure>>>(device_data->pairwise_twists, device_data->eval_point_twists, device_data->num_eval_points, device_data->num_stokeslets);
    err = cudaGetLastError();
    if (err != cudaSuccess)
    {
      printf("Kernel launch failed with error: %s\n", cudaGetErrorString(err));
    }

    cudaMemcpy(eval_point_twists, device_data->eval_point_twists, device_data->eval_point_twists_size, cudaMemcpyDeviceToHost);

    cudaDeviceSynchronize();
  }

  void fluid_finalize(FluidDeviceData *device_data)
  {
    cudaFree(device_data->eval_points);
    cudaFree(device_data->stokeslets);
    cudaFree(device_data->pairwise_twists);
    cudaFree(device_data->eval_point_twists);
    free(device_data);
  }
}
