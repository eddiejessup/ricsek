#include <linear_fields.h>
#include <stdio.h>

__constant__ double d_lx;
__constant__ double d_ly;
__constant__ double d_lz;

__device__ double apply_periodic_boundary(double d, double length)
{
  if (d > 0.5f * length)
    return d - length;
  else if (d < -0.5f * length)
    return d + length;
  else
    return d;
}

__global__ void pairwiseDistance(double *positions, double *distances, unsigned long n)
{
  unsigned long i = blockIdx.x * blockDim.x + threadIdx.x;
  unsigned long j = blockIdx.y * blockDim.y + threadIdx.y;
  // printf("i=%lu, j=%lu, blockIdx.x=%i, blockIdx.y=%i, threadIdx.x=%i, threadIdx.y=%i, blockDim.x=%i, blockDim.y=%i\n", i, j, blockIdx.x, blockIdx.y, threadIdx.x, threadIdx.y, blockDim.x, blockDim.y);

  if (i < n && j < n)
  {
    unsigned long pos_i_idx = i * 3;
    unsigned long pos_j_idx = j * 3;

    double dx = apply_periodic_boundary(positions[pos_i_idx] - positions[pos_j_idx], d_lx);
    double dy = apply_periodic_boundary(positions[pos_i_idx + 1] - positions[pos_j_idx + 1], d_ly);
    double dz = apply_periodic_boundary(positions[pos_i_idx + 2] - positions[pos_j_idx + 2], d_lz);

    unsigned long dr_idx = pos_i_idx * n + pos_j_idx;

    printf("i=%lu, j=%lu, dx=%f, dy=%f, dz=%f, dr_idx=%lu\n", i, j, dx, dy, dz, dr_idx);

    distances[dr_idx] = dx;
    distances[dr_idx + 1] = dy;
    distances[dr_idx + 2] = dz;
  }
}

__global__ void netForce(double *distances, double *forces, unsigned long n)
{
  unsigned long i = blockIdx.x * blockDim.x + threadIdx.x;

  if (i < n)
  {
    double fx = 0.0f;
    double fy = 0.0f;
    double fz = 0.0f;

    unsigned long i_idx = i * 3;
    for (unsigned long j = 0; j < n; j++)
    {
      if (j != i)
      {
        unsigned long dr_idx = i_idx * n + 3 * j;

        double dx = distances[dr_idx];
        double dy = distances[dr_idx + 1];
        double dz = distances[dr_idx + 2];

        double r = sqrtf(dx * dx + dy * dy + dz * dz);
        double r3_inv = 1.0 / (r * r * r);

        fx += dx * r3_inv;
        fy += dy * r3_inv;
        fz += dz * r3_inv;
        printf("FORCES: i=%lu, j=%lu, fx=%f, fy=%f, fz=%f, dr_idx=%lu\n", i, j, fx, fy, fz, dr_idx);
      }
    }

    forces[i_idx] = fx;
    forces[i_idx + 1] = fy;
    forces[i_idx + 2] = fz;
  }
}

extern "C"
{

  DeviceData *init(unsigned long num_elements, double lx, double ly, double lz)
  {
    DeviceData *device_data = (DeviceData *)malloc(sizeof(DeviceData));

    cudaMemcpyToSymbol(d_lx, &lx, sizeof(double));
    cudaMemcpyToSymbol(d_ly, &ly, sizeof(double));
    cudaMemcpyToSymbol(d_lz, &lz, sizeof(double));

    unsigned long numValues = 3 * num_elements;

    unsigned long positions_size = numValues * sizeof(double);
    double *dPositions;
    cudaMalloc((void **)&dPositions, positions_size);
    device_data->positions = dPositions;
    device_data->positions_size = positions_size;

    unsigned long distances_size = numValues * numValues * sizeof(double);
    double *d_distances;
    cudaMalloc((void **)&d_distances, distances_size);
    device_data->distances = d_distances;
    device_data->distances_size = distances_size;

    double *d_forces;
    cudaMalloc((void **)&d_forces, positions_size);
    device_data->forces = d_forces;

    device_data->num_elements = num_elements;
    return device_data;
  }

  void populatePairwiseDistances(DeviceData *device_data, double *positions, unsigned int threadsPerBlockAxis)
  {
    unsigned long num_elements = device_data->num_elements;
    dim3 block_structure = dim3(threadsPerBlockAxis, threadsPerBlockAxis);
    unsigned int blocks_per_grid_axis = (num_elements + threadsPerBlockAxis - 1) / threadsPerBlockAxis;
    dim3 grid_structure = dim3(blocks_per_grid_axis, blocks_per_grid_axis);
    printf("Grid shape: %d, %d\n", grid_structure.x, grid_structure.y);
    printf("Block shape: %d, %d\n", block_structure.x, block_structure.y);
    // Print first 6 elements of positions.
    printf("First 6 elements of positions:\n");
    for (int i = 0; i < 6; i++)
    {
      printf("%f\n", positions[i]);
    }
    cudaMemcpy(device_data->positions, positions, device_data->positions_size, cudaMemcpyHostToDevice);
    // <<grid structure, block structure>>>
    pairwiseDistance<<<grid_structure, block_structure>>>(device_data->positions, device_data->distances, num_elements);

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess)
    {
      printf("Kernel launch failed with error: %s\n", cudaGetErrorString(err));
    }

    // Synchronize.
    cudaDeviceSynchronize();
  }

  void fetchPairwiseDistances(DeviceData *device_data, double *distances)
  {
    cudaMemcpy(distances, device_data->distances, device_data->distances_size, cudaMemcpyDeviceToHost);
    cudaDeviceSynchronize();
  }

  void netForces(DeviceData *device_data, double *forces, unsigned int threadsPerBlockAxis)
  {
    unsigned long num_elements = device_data->num_elements;
    unsigned int blocks_per_grid_axis = (num_elements + threadsPerBlockAxis - 1) / threadsPerBlockAxis;
    dim3 grid_shape = dim3(blocks_per_grid_axis);
    dim3 block_shape = dim3(threadsPerBlockAxis);
    netForce<<<grid_shape, block_shape>>>(device_data->distances, device_data->forces, num_elements);
    cudaMemcpy(forces, device_data->forces, device_data->positions_size, cudaMemcpyDeviceToHost);
  }

  void finalize(DeviceData *device_data)
  {
    cudaFree(device_data->positions);
    cudaFree(device_data->distances);
    cudaFree(device_data->forces);
    free(device_data);
  }
}
