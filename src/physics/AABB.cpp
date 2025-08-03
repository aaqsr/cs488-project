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

bool AABB::contains(const linalg::aliases::float3& pt) const
{
    return (pt.x >= min.x && pt.x <= max.x) &&
           (pt.y >= min.y && pt.y <= max.y) && (pt.z >= min.z && pt.z <= max.z);
}

bool AABB::contains(const AABB& other) const
{
    return (other.min.x >= min.x && other.max.x <= max.x) &&
           (other.min.y >= min.y && other.max.y <= max.y) &&
           (other.min.z >= min.z && other.max.z <= max.z);
}

std::array<linalg::aliases::float3, 8> AABB::getCorners() const
{

    using linalg::aliases::float3;
    std::array<float3, 8> corners = {
      float3{min.x, min.y, min.z},
      float3{max.x, min.y, min.z},
      float3{min.x, max.y, min.z},
      float3{max.x, max.y, min.z},
      float3{min.x, min.y, max.z},
      float3{max.x, min.y, max.z},
      float3{min.x, max.y, max.z},
      float3{max.x, max.y, max.z}
    };
    return corners;
}

AABB AABB::computeMovedAndScaledAABB(
  const linalg::aliases::float3& displacement,
  const linalg::aliases::float3x3& rotationMatrix) const
{
    linalg::aliases::float3 currCentre = (max + min) * 0.5F;
    linalg::aliases::float3 currExtents = (max - min) * 0.5F;

    linalg::aliases::float3 newCentre =
      linalg::mul(rotationMatrix, currCentre) + displacement;

    // Compute world extents using absolute values of rotation matrix.
    // This gives us the maximum extent in each world axis. I think.
    linalg::aliases::float3 worldExtents = linalg::mul(linalg::abs(rotationMatrix), currExtents);

    return AABB{newCentre - worldExtents, newCentre + worldExtents};
}
