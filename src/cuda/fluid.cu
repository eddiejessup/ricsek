#include <stdio.h>
#include <cuda_ricsek.h>
#include <common.cu>

__constant__ BoundaryConfig d_bc;

__global__ void evaluateSingularity(ObjectPoint *eval_points, Singularity *singularities, V3Pair *pairwise_twists, unsigned int num_eval_points, unsigned int num_singularities)
{
  unsigned int eval_i = blockIdx.x * blockDim.x + threadIdx.x;
  unsigned int singularity_i = blockIdx.y * blockDim.y + threadIdx.y;

  if (eval_i < num_eval_points && singularity_i < num_singularities)
  {
    ObjectPoint eval_point = eval_points[eval_i];
    Singularity singularity = singularities[singularity_i];

    unsigned int pair_idx = eval_i * num_singularities + singularity_i;
    if (eval_point.object_id == singularity.object_point.object_id)
    {
      pairwise_twists[pair_idx] = zero_v3_pair();
      return;
    }

    // r is the vector pointing from the singularity to the point of interest.
    V3 r = minimum_image_vector_objs(eval_point, singularity.object_point, d_bc);

    float r1 = length(r);
    float r1_inv = 1.0 / r1;
    float r3_inv = r1_inv * r1_inv * r1_inv;

    V3 v = zero_v3();
    if (singularity.singularity_type == STOKESLET)
    {
      // (a / |r|) + (a.dot(r) * r / |r|^3)
      float r_coeff = dot(singularity.strength, r) * r3_inv;
      v = V3{
          (r1_inv * singularity.strength.x) + (r_coeff * r.x),
          (r1_inv * singularity.strength.y) + (r_coeff * r.y),
          (r1_inv * singularity.strength.z) + (r_coeff * r.z)};
    }
    else if (singularity.singularity_type == ROTLET)
    {
      v = scale(cross(singularity.strength, r), r3_inv);
    }
    pairwise_twists[pair_idx].a = v;
  }
}

__global__ void gatherSingularities(V3Pair *pairwise_twists, V3Pair *eval_point_twists, unsigned int num_eval_points, unsigned int num_singularities)
{
  unsigned int eval_i = blockIdx.x * blockDim.x + threadIdx.x;

  if (eval_i < num_eval_points)
  {
    V3 v_tot = zero_v3();
    V3 vort_tot = zero_v3();

    for (unsigned int singularity_i = 0; singularity_i < num_singularities; singularity_i++)
    {
      unsigned int pair_idx = eval_i * num_singularities + singularity_i;

      V3 v = pairwise_twists[pair_idx].a;
      v_tot.x += v.x;
      v_tot.y += v.y;
      v_tot.z += v.z;

      V3 vort = pairwise_twists[pair_idx].b;
      vort_tot.x += vort.x;
      vort_tot.y += vort.y;
      vort_tot.z += vort.z;
    }
    eval_point_twists[eval_i] = V3Pair{v_tot, vort_tot};
  }
}

extern "C"
{

  FluidDeviceData *fluid_init(unsigned int num_eval_points, unsigned int num_singularities, BoundaryConfig bc)
  {
    FluidDeviceData *device_data = (FluidDeviceData *)malloc(sizeof(FluidDeviceData));

    cudaMemcpyToSymbol(d_bc, &bc, sizeof(BoundaryConfig));

    unsigned int eval_points_size = num_eval_points * sizeof(ObjectPoint);
    ObjectPoint *d_eval_points;
    cudaMalloc((void **)&d_eval_points, eval_points_size);
    device_data->eval_points = d_eval_points;
    device_data->eval_points_size = eval_points_size;
    device_data->num_eval_points = num_eval_points;

    unsigned int singularities_size = num_singularities * sizeof(Singularity);
    Singularity *d_singularities;
    cudaMalloc((void **)&d_singularities, singularities_size);
    device_data->singularities = d_singularities;
    device_data->singularities_size = singularities_size;
    device_data->num_singularities = num_singularities;

    unsigned int pairwise_twists_size = num_eval_points * num_singularities * sizeof(V3Pair);
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

  void fluid_evaluate(FluidDeviceData *device_data, ObjectPoint *eval_points, Singularity *singularities, V3Pair *eval_point_twists, unsigned int threads_per_block_axis)
  {
    unsigned int num_blocks_x = (device_data->num_eval_points + threads_per_block_axis - 1) / threads_per_block_axis;
    unsigned int num_blocks_y = (device_data->num_singularities + threads_per_block_axis - 1) / threads_per_block_axis;

    dim3 block_structure = dim3(threads_per_block_axis, threads_per_block_axis);
    dim3 grid_structure = dim3(
        num_blocks_x,
        num_blocks_y);
    // printf("Grid shape: %d, %d\n", grid_structure.x, grid_structure.y);
    // printf("Block shape: %d, %d\n", block_structure.x, block_structure.y);
    cudaMemcpy(device_data->eval_points, eval_points, device_data->eval_points_size, cudaMemcpyHostToDevice);
    cudaMemcpy(device_data->singularities, singularities, device_data->singularities_size, cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();

    evaluateSingularity<<<grid_structure, block_structure>>>(
        device_data->eval_points, device_data->singularities,
        device_data->pairwise_twists,
        device_data->num_eval_points, device_data->num_singularities);
    cudaDeviceSynchronize();
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess)
    {
      printf("Kernel '%s' launch failed with error: %s\n", "evaluateSingularity", cudaGetErrorString(err));
    }

    dim3 gather_block_structure = dim3(threads_per_block_axis);
    dim3 gather_grid_structure = dim3(num_blocks_x);
    gatherSingularities<<<gather_grid_structure, gather_block_structure>>>(device_data->pairwise_twists, device_data->eval_point_twists, device_data->num_eval_points, device_data->num_singularities);
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
    cudaFree(device_data->singularities);
    cudaFree(device_data->pairwise_twists);
    cudaFree(device_data->eval_point_twists);
    free(device_data);
  }
}
