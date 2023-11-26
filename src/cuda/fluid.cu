#include <stdio.h>
#include <cuda_ricsek.h>
#include <common.cu>

__constant__ BoundaryConfig d_bc;

__global__ void evaluateStokeslet(ObjectPoint *eval_points, Stokeslet *stokeslets, V3Pair *pairwise_twists, unsigned int num_eval_points, unsigned int num_stokeslets)
{
  unsigned int eval_i = blockIdx.x * blockDim.x + threadIdx.x;
  unsigned int stokeslet_i = blockIdx.y * blockDim.y + threadIdx.y;

  if (eval_i < num_eval_points && stokeslet_i < num_stokeslets)
  {
    ObjectPoint eval_point = eval_points[eval_i];
    Stokeslet stokeslet = stokeslets[stokeslet_i];

    unsigned int pair_idx = eval_i * num_stokeslets + stokeslet_i;
    // printf("EVAL: eval_i=%u, stokeslet_i=%u, pair_idx=%u\n", eval_i, stokeslet_i, pair_idx);
    if (eval_point.object_id == stokeslet.object_point.object_id)
    {
      pairwise_twists[pair_idx] = zero_v3_pair();
      return;
    }

    // r is the vector pointing from the singularity to the point of interest.
    V3 r = apply_boundary_conditions(sub(eval_point.position, stokeslet.object_point.position), d_bc);

    float r1 = length(r);
    float r1_inv = 1.0 / r1;
    float r3_inv = r1_inv * r1_inv * r1_inv;

    // (a / |r|) + (a.dot(r) * r / |r|^3)
    float r_coeff = dot(stokeslet.force, r) * r3_inv;

    // if (r_naive.x != r.x) {
    //   printf("eval_point.position.x=%f, stokeslet.x=%f, r_naive.x=%f, r.x=%f, r1=%f, r1_inv=%f, r3_inv=%f, r_coeff=%f\n", eval_point.position.x, stokeslet.object_point.position.x, r_naive.x, r.x, r1, r1_inv, r3_inv, r_coeff);
    // }

    pairwise_twists[pair_idx].a = V3{
        (r1_inv * stokeslet.force.x) + (r_coeff * r.x),
        (r1_inv * stokeslet.force.y) + (r_coeff * r.y),
        (r1_inv * stokeslet.force.z) + (r_coeff * r.z)};
  }
}

__global__ void gatherSingularities(V3Pair *pairwise_twists, V3Pair *eval_point_twists, unsigned int num_eval_points, unsigned int num_stokeslets)
{
  unsigned int eval_i = blockIdx.x * blockDim.x + threadIdx.x;

  if (eval_i < num_eval_points)
  {
    V3 v_tot = zero_v3();
    V3 vort_tot = zero_v3();

    for (unsigned int stokeslet_i = 0; stokeslet_i < num_stokeslets; stokeslet_i++)
    {
      unsigned int pair_idx = eval_i * num_stokeslets + stokeslet_i;

      V3 v = pairwise_twists[pair_idx].a;
      // printf("GATHER: eval_i=%u, stokeslet_i=%u, v=(%f, %f, %f)\n", eval_i, stokeslet_i, pairwise_twists[pair_idx].x, pairwise_twists[pair_idx].y, pairwise_twists[pair_idx].z);
      v_tot.x += v.x;
      v_tot.y += v.y;
      v_tot.z += v.z;

      V3 vort = pairwise_twists[pair_idx].b;
      vort_tot.x += vort.x;
      vort_tot.y += vort.y;
      vort_tot.z += vort.z;
    }
    eval_point_twists[eval_i] = V3Pair{v_tot, vort_tot};

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
    // printf("Allocating %u bytes for eval_points\n", eval_points_size);
    ObjectPoint *d_eval_points;
    cudaMalloc((void **)&d_eval_points, eval_points_size);
    device_data->eval_points = d_eval_points;
    device_data->eval_points_size = eval_points_size;
    device_data->num_eval_points = num_eval_points;

    unsigned int stokeslets_size = num_stokeslets * sizeof(Stokeslet);
    // printf("Allocating %u bytes for stokeslets\n", stokeslets_size);
    Stokeslet *d_stokeslets;
    cudaMalloc((void **)&d_stokeslets, stokeslets_size);
    device_data->stokeslets = d_stokeslets;
    device_data->stokeslets_size = stokeslets_size;
    device_data->num_stokeslets = num_stokeslets;

    unsigned int pairwise_twists_size = num_eval_points * num_stokeslets * sizeof(V3Pair);
    // printf("Allocating %u bytes for pairwise_twists\n", pairwise_twists_size);
    V3Pair *d_pairwise_twists;
    cudaMalloc((void **)&d_pairwise_twists, pairwise_twists_size);
    device_data->pairwise_twists = d_pairwise_twists;
    device_data->pairwise_twists_size = pairwise_twists_size;

    unsigned int eval_point_twists_size = num_eval_points * sizeof(V3Pair);
    // printf("Allocating %u bytes for eval_point_twists\n", eval_point_twists_size);
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
    cudaDeviceSynchronize();

    evaluateStokeslet<<<grid_structure, block_structure>>>(
        device_data->eval_points, device_data->stokeslets,
        device_data->pairwise_twists,
        device_data->num_eval_points, device_data->num_stokeslets);
    cudaDeviceSynchronize();
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess)
    {
      printf("Kernel '%s' launch failed with error: %s\n", "evaluateStokeslet", cudaGetErrorString(err));
    }

    dim3 gather_block_structure = dim3(threads_per_block_axis);
    dim3 gather_grid_structure = dim3(num_blocks_x);
    gatherSingularities<<<gather_grid_structure, gather_block_structure>>>(device_data->pairwise_twists, device_data->eval_point_twists, device_data->num_eval_points, device_data->num_stokeslets);
    cudaDeviceSynchronize();
    err = cudaGetLastError();
    if (err != cudaSuccess)
    {
      printf("Kernel '%s' launch failed with error: %s\n", "gatherSingularities", cudaGetErrorString(err));
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
