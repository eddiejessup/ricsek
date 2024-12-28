#pragma once

#include <stdbool.h>

typedef struct
{
  float x;
  float y;
  float z;
} V3;

typedef struct
{
  V3 lengths;
  bool closed_x;
  bool closed_y;
  bool closed_z;
} BoundaryConfig;

typedef struct
{
  unsigned int object_id;
  V3 position_com;
  V3 position;
} ObjectPoint;

typedef struct
{
  V3 a;
  V3 b;
} V3Pair;

// Electro.

typedef struct
{
  V3Pair *segments;
  unsigned int num_segments;
  unsigned int segments_size;
  V3Pair *pairwise_wrenches;
  unsigned int pairwise_wrenches_size;
  V3Pair *net_wrenches;
  unsigned int net_wrenches_size;
} ElectroDeviceData;

#ifdef __cplusplus
extern "C"
{
#endif
  ElectroDeviceData *electro_init(unsigned int num_segments, BoundaryConfig bc, float radius);

  void electro_evaluate(ElectroDeviceData *device_data, V3Pair *segments, V3Pair *net_wrenches, unsigned int threads_per_block_axis, unsigned int num_points);

  void electro_finalize(ElectroDeviceData *device_data);
#ifdef __cplusplus
}
#endif

// Fluid

typedef enum
{
  STOKESLET,
  STOKES_DOUBLET,
  ROTLET,
  STRESSLET,
  POTENTIAL_DOUBLET,
} SingularityType;

typedef union
  {
    V3 strength;
    V3Pair components;
} SingularityParams;

typedef struct
{
  ObjectPoint object_point;
  SingularityType singularity_type;
  SingularityParams params;
} Singularity;

typedef struct
{
  ObjectPoint *eval_points;
  unsigned int num_eval_points;
  unsigned int eval_points_size;
  Singularity *singularities;
  unsigned int num_singularities;
  unsigned int singularities_size;
  V3Pair *pairwise_twists;
  unsigned int pairwise_twists_size;
  V3Pair *eval_point_twists;
  unsigned int eval_point_twists_size;
} FluidDeviceData;

#ifdef __cplusplus
extern "C"
{
#endif
  FluidDeviceData *fluid_init(unsigned int num_eval_points, unsigned int num_singularities, BoundaryConfig bc);

  void fluid_evaluate(FluidDeviceData *device_data, ObjectPoint *eval_points, Singularity *singularities, V3Pair *eval_point_twists, unsigned int threads_per_block_axis);

  void fluid_finalize(FluidDeviceData *device_data);

#ifdef __cplusplus
}
#endif
