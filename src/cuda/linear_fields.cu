#include <linear_fields.h>
#include <stdio.h>

__constant__ float d_lx;
__constant__ float d_ly;
__constant__ float d_lz;

__device__ float apply_periodic_boundary(float d, float length)
{
  if (d > 0.5f * length)
    return d - length;
  else if (d < -0.5f * length)
    return d + length;
  else
    return d;
}

__device__ float dot(V3 a, V3 b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

__global__ void evaluateStokeslet(ObjectPoint *eval_points, Stokeslet *stokeslets, V3 *pairwise_velocities, unsigned int num_eval_points, unsigned int num_stokeslets)
{
  unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
  unsigned int j = blockIdx.y * blockDim.y + threadIdx.y;

  if (i < num_eval_points && j < num_stokeslets)
  {
    // printf("i=%u, j=%u\n", i, j);

    ObjectPoint eval_point_i = eval_points[i];
    Stokeslet stokeslet_j = stokeslets[j];

    unsigned int pair_idx = i * num_eval_points + j;
    if (eval_point_i.object_id == stokeslet_j.object_point.object_id)
    {
      pairwise_velocities[pair_idx] = V3{0.0f, 0.0f, 0.0f};
      return;
    }

    // r is the vector pointing from the singularity to the point of interest.
    V3 r = V3{
        apply_periodic_boundary(eval_point_i.position.x - stokeslet_j.object_point.position.x, d_lx),
        apply_periodic_boundary(eval_point_i.position.y - stokeslet_j.object_point.position.y, d_ly),
        apply_periodic_boundary(eval_point_i.position.z - stokeslet_j.object_point.position.z, d_lz)};

    float r1 = sqrtf(r.x * r.x + r.y * r.y + r.z * r.z);
    float r1_inv = 1.0 / r1;
    float r3_inv = 1.0 / (r1 * r1 * r1);

    // (a / r1_inv) + (a.dot(r) * r / r1_inv.powi(3))
    float r_coeff = dot(stokeslet_j.force, r) * r3_inv;
    pairwise_velocities[pair_idx] = V3{
        (r1_inv * stokeslet_j.force.x) + (r_coeff * r.x),
        (r1_inv * stokeslet_j.force.y) + (r_coeff * r.y),
        (r1_inv * stokeslet_j.force.z) + (r_coeff * r.z)};
    // printf("i=%u, j=%u, r1=%f, r1_inv=%f, r3_inv=%f, r=(%f, %f, %f), v=(%f, %f, %f)\n", i, j, r1, r1_inv, r3_inv, r.x, r.y, r.z, pairwise_velocities[pair_idx].x, pairwise_velocities[pair_idx].y, pairwise_velocities[pair_idx].z);
  }
}

__global__ void gatherSingularities(V3 *pairwise_velocities, V3 *eval_point_velocities, unsigned int num_eval_points, unsigned int num_stokeslets)
{
  unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;

  if (i < num_eval_points)
  {
    V3 v_tot = V3{0.0f, 0.0f, 0.0f};

    for (unsigned int j = 0; j < num_stokeslets; j++)
    {
      unsigned int pair_idx = i * num_eval_points + j;

      V3 v = pairwise_velocities[pair_idx];
      // printf("GATHER: i=%u, j=%u, v=(%f, %f, %f)\n", i, j, pairwise_velocities[pair_idx].x, pairwise_velocities[pair_idx].y, pairwise_velocities[pair_idx].z);
      v_tot.x += v.x;
      v_tot.y += v.y;
      v_tot.z += v.z;
    }
    eval_point_velocities[i] = v_tot;

    // printf("GATHER-FINAL: i=%u, v_tot=(%f, %f, %f), eval_point_velocities[i]=(%f, %f, %f)\n", i, v_tot.x, v_tot.y, v_tot.z, eval_point_velocities[i].x, eval_point_velocities[i].y, eval_point_velocities[i].z);
  }
}

extern "C"
{

  DeviceData *init(unsigned int num_eval_points, unsigned int num_stokeslets, float lx, float ly, float lz)
  {
    DeviceData *device_data = (DeviceData *)malloc(sizeof(DeviceData));

    cudaMemcpyToSymbol(d_lx, &lx, sizeof(float));
    cudaMemcpyToSymbol(d_ly, &ly, sizeof(float));
    cudaMemcpyToSymbol(d_lz, &lz, sizeof(float));

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

    unsigned int pairwise_velocities_size = num_eval_points * num_stokeslets * sizeof(V3);
    V3 *d_pairwise_velocities;
    cudaMalloc((void **)&d_pairwise_velocities, pairwise_velocities_size);
    device_data->pairwise_velocities = d_pairwise_velocities;
    device_data->pairwise_velocities_size = pairwise_velocities_size;

    unsigned int eval_point_velocities_size = num_eval_points * sizeof(V3);
    V3 *d_eval_point_velocities;
    cudaMalloc((void **)&d_eval_point_velocities, eval_point_velocities_size);
    device_data->eval_point_velocities = d_eval_point_velocities;
    device_data->eval_point_velocities_size = eval_point_velocities_size;
    return device_data;
  }

  void evaluateStokeslets(DeviceData *device_data, ObjectPoint *eval_points, Stokeslet *stokeslets, V3 *eval_point_velocities, unsigned int threadsPerBlockAxis)
  {
    unsigned int num_blocks_x = (device_data->num_eval_points + threadsPerBlockAxis - 1) / threadsPerBlockAxis;
    unsigned int num_blocks_y = (device_data->num_stokeslets + threadsPerBlockAxis - 1) / threadsPerBlockAxis;

    dim3 block_structure = dim3(threadsPerBlockAxis, threadsPerBlockAxis);
    dim3 grid_structure = dim3(
        num_blocks_x,
        num_blocks_y);
    // printf("Grid shape: %d, %d\n", grid_structure.x, grid_structure.y);
    // printf("Block shape: %d, %d\n", block_structure.x, block_structure.y);
    cudaMemcpy(device_data->eval_points, eval_points, device_data->eval_points_size, cudaMemcpyHostToDevice);
    cudaMemcpy(device_data->stokeslets, stokeslets, device_data->stokeslets_size, cudaMemcpyHostToDevice);
    evaluateStokeslet<<<grid_structure, block_structure>>>(
        device_data->eval_points, device_data->stokeslets,
        device_data->pairwise_velocities,
        device_data->num_eval_points, device_data->num_stokeslets);

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess)
    {
      printf("Kernel launch failed with error: %s\n", cudaGetErrorString(err));
    }

    dim3 gather_block_structure = dim3(threadsPerBlockAxis);
    dim3 gather_grid_structure = dim3(num_blocks_x);
    gatherSingularities<<<gather_grid_structure, gather_block_structure>>>(device_data->pairwise_velocities, device_data->eval_point_velocities, device_data->num_eval_points, device_data->num_stokeslets);
    err = cudaGetLastError();
    if (err != cudaSuccess)
    {
      printf("Kernel launch failed with error: %s\n", cudaGetErrorString(err));
    }

    cudaMemcpy(eval_point_velocities, device_data->eval_point_velocities, device_data->eval_point_velocities_size, cudaMemcpyDeviceToHost);

    cudaDeviceSynchronize();
  }

  void finalize(DeviceData *device_data)
  {
    cudaFree(device_data->eval_points);
    cudaFree(device_data->stokeslets);
    cudaFree(device_data->pairwise_velocities);
    cudaFree(device_data->eval_point_velocities);
    free(device_data);
  }
}
