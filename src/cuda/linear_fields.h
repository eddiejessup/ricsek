#pragma once

typedef struct
{
  double *positions;
  unsigned long positions_size;
  double *distances;
  unsigned long distances_size;
  double *forces;
  unsigned long num_elements;
} DeviceData;

#ifdef __cplusplus
extern "C"
{
#endif
  DeviceData *init(unsigned long num_elements, double lx, double ly, double lz);

  void populatePairwiseDistances(DeviceData *device_data, double *positions, unsigned int threadsPerBlock);

  void fetchPairwiseDistances(DeviceData *device_data, double *distances);

  void netForces(DeviceData *device_data, double *forces, unsigned int threadsPerBlockAxis);

  void finalize(DeviceData *device_data);
#ifdef __cplusplus
}
#endif
