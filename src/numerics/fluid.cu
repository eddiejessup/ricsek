#include <stdio.h>
#include "cuda_interface.h"
#include "cuda_common.h"

__constant__ BoundaryConfig d_bc;

__host__ __device__ V3 evaluateStokesletV(V3 strength, V3 r, float r_length)
{
  // (a / |r|) + (a.dot(r) * r / |r|^3)
  V3 v_f = scale(strength, 1.0 / r_length);
  V3 v_r = scale(r, dot(strength, r) / (r_length * r_length * r_length));
  return add(v_f, v_r);
}

__host__ __device__ V3 evaluateRotletV(V3 strength, V3 r, float r_length)
{
  return scale(cross(strength, r), 1.0 / (r_length * r_length * r_length));
}

__host__ __device__ V3 evaluateStressletV(V3 a, V3 b, V3 r, float r_length)
{
  float term_1 = -dot(a, b) / (r_length * r_length * r_length);
  float term_2 = 3.0 * dot(a, r) * dot(b, r) / (r_length * r_length * r_length * r_length * r_length);
  return scale(r, term_1 + term_2);
}

__host__ __device__ V3 evaluatePotentialDoubletV(V3 d, V3 r, float r_length)
{
  V3 v_1 = scale(d, -1.0 / (r_length * r_length * r_length));
  V3 v_2 = scale(r, 3.0 * dot(d, r) / (r_length * r_length * r_length * r_length * r_length));
  return add(v_1, v_2);
}

__host__ __device__ V3 evaluateStokesDoubletV(V3 a, V3 b, V3 r, float r_length)
{
  V3 c = cross(a, b);
  V3 v_rot = evaluateRotletV(c, r, r_length);
  V3 v_stress = evaluateStressletV(a, b, r, r_length);
  return add(v_rot, v_stress);
}

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

    float r_length = length(r);
    if (isnan(r_length) || isinf(r_length) || r_length < 1e-6)
    {
      printf("Warning: NaN or infinite r_length detected. r=(%f, %f, %f), r_length=%f\n", r.x, r.y, r.z, r_length);
    };

    V3 v = zero_v3();
    if (singularity.singularity_type == STOKESLET)
    {
      v = evaluateStokesletV(singularity.params.strength, r, r_length);
    }
    else if (singularity.singularity_type == STOKES_DOUBLET)
    {
      v = evaluateStokesDoubletV(singularity.params.components.a, singularity.params.components.b, r, r_length);
    }
    else if (singularity.singularity_type == ROTLET)
    {
      v = evaluateRotletV(singularity.params.strength, r, r_length);
    }
    else if (singularity.singularity_type == STRESSLET)
    {
      v = evaluateStressletV(singularity.params.components.a, singularity.params.components.b, r, r_length);
    }
    else if (singularity.singularity_type == POTENTIAL_DOUBLET)
    {
      v = evaluatePotentialDoubletV(singularity.params.strength, r, r_length);
    }
    else
    {
      printf("Unknown singularity type: %d\n", singularity.singularity_type);
    }
    // Check if any component of v is NaN or infinite, and print a warning if so.
    if (isnan(v.x) || isnan(v.y) || isnan(v.z) || isinf(v.x) || isinf(v.y) || isinf(v.z))
    {
      printf("Warning: NaN or infinite velocity component detected. Singularity type: %d\n", singularity.singularity_type);
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
