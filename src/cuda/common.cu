#include <cuda_ricsek.h>

__host__ __device__ float apply_periodic_boundary(float d, float length)
{
  if (d > 0.5f * length)
    return d - length;
  else if (d <= -0.5f * length)
    return d + length;
  else
    return d;
}

__host__ __device__ V3 apply_boundary_conditions(V3 d, BoundaryConfig bc)
{
  if (!bc.closed_x)
  {
    d.x = apply_periodic_boundary(d.x, bc.lengths.x);
  }
  if (!bc.closed_y)
  {
    d.y = apply_periodic_boundary(d.y, bc.lengths.y);
  }
  if (!bc.closed_z)
  {
    d.z = apply_periodic_boundary(d.z, bc.lengths.z);
  }
  return d;
}

__host__ __device__ V3 sub(V3 a, V3 b)
{
  return V3{a.x - b.x, a.y - b.y, a.z - b.z};
}

__host__ __device__ V3 add(V3 a, V3 b)
{
  return V3{a.x + b.x, a.y + b.y, a.z + b.z};
}

__host__ __device__ V3 scale(V3 a, float b)
{
  return V3{a.x * b, a.y * b, a.z * b};
}

__host__ __device__ V3 minus(V3 a)
{
  return scale(a, -1.0f);
}

__host__ __device__ V3 cross(V3 a, V3 b)
{
  return V3{
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x};
}

__host__ __device__ float dot(V3 a, V3 b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

__host__ __device__ float length_sq(V3 a)
{
  return dot(a, a);
}

__host__ __device__ float length(V3 a)
{
  return sqrtf(length_sq(a));
}

__host__ __device__ V3 unit(V3 a)
{
  float l = length(a);
  return V3{a.x / l, a.y / l, a.z / l};
}

__host__ __device__ V3 zero_v3()
{
  return V3{0.0f, 0.0f, 0.0f};
}

__host__ __device__ V3Pair zero_v3_pair()
{
  return V3Pair{zero_v3(), zero_v3()};
}
