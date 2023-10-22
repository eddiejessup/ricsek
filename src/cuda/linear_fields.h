#pragma once

typedef struct
{
  float x;
  float y;
  float z;
} V3;

typedef struct
{
  unsigned int object_id;
  V3 position;
} ObjectPoint;

typedef struct
{
  ObjectPoint object_point;
  V3 force;
} Stokeslet;

typedef struct
{
  ObjectPoint *eval_points;
  unsigned int num_eval_points;
  unsigned int eval_points_size;
  Stokeslet *stokeslets;
  unsigned int num_stokeslets;
  unsigned int stokeslets_size;
  V3 *pairwise_velocities;
  unsigned int pairwise_velocities_size;
  V3 *eval_point_velocities;
  unsigned int eval_point_velocities_size;
} DeviceData;

#ifdef __cplusplus
extern "C"
{
#endif
  DeviceData *init(unsigned int num_eval_points, unsigned int num_stokeslets, float lx, float ly, float lz);

  void evaluateStokeslets(DeviceData *device_data, ObjectPoint *eval_points, Stokeslet *stokeslets, V3 *eval_point_velocities, unsigned int threadsPerBlockAxis);

  void finalize(DeviceData *device_data);
#ifdef __cplusplus
}
#endif
