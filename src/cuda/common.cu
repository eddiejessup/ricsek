#include <cuda_ricsek.h>

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

__host__ __device__ float apply_periodic_boundary_obj(float d_com, float d, float length)
{
  if (d_com > 0.5f * length)
    return d - length;
  else if (d_com < -0.5f * length)
    return d + length;
  else
    return d;
}

__host__ __device__ V3 apply_boundary_conditions_obj(V3 r_com, V3 r, BoundaryConfig bc)
{
  if (!bc.closed_x)
  {
    r.x = apply_periodic_boundary_obj(r_com.x, r.x, bc.lengths.x);
  }
  if (!bc.closed_y)
  {
    r.y = apply_periodic_boundary_obj(r_com.y, r.y, bc.lengths.y);
  }
  if (!bc.closed_z)
  {
    r.z = apply_periodic_boundary_obj(r_com.z, r.z, bc.lengths.z);
  }
  return r;
}

__host__ __device__ V3 apply_boundary_conditions_point(V3 r, BoundaryConfig bc)
{
  return apply_boundary_conditions_obj(r, r, bc);
}

__host__ __device__ V3 minimum_image_vector_objs(ObjectPoint p_to, ObjectPoint p_from, BoundaryConfig bc)
{
  V3 r_com = sub(p_to.position_com, p_from.position_com);
  V3 r = sub(p_to.position, p_from.position);
  return apply_boundary_conditions_obj(r_com, r, bc);
}
