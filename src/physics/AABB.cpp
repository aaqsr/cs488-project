#include "physics/AABB.hpp"

AABB::AABB(const linalg::aliases::float3& min,
           const linalg::aliases::float3& max)
  : min{min}, max{max}
{
}
AABB::AABB() : min{0.0F}, max{0.0F}
{
}

bool AABB::overlaps(const AABB& other) const
{
    return (min.x <= other.max.x && max.x >= other.min.x) &&
           (min.y <= other.max.y && max.y >= other.min.y) &&
           (min.z <= other.max.z && max.z >= other.min.z);
}

void AABB::expand(const linalg::aliases::float3& point)
{
    min.x = std::min(min.x, point.x);
    min.y = std::min(min.y, point.y);
    min.z = std::min(min.z, point.z);
    max.x = std::max(max.x, point.x);
    max.y = std::max(max.y, point.y);
    max.z = std::max(max.z, point.z);
}
