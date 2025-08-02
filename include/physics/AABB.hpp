#pragma once

#include <algorithm>
#include <linalg.h>

struct AABB
{
    linalg::aliases::float3 min;
    linalg::aliases::float3 max;

    AABB();

    AABB(const linalg::aliases::float3& min,
         const linalg::aliases::float3& max);

    // any non-empty intersection
    [[nodiscard]] bool overlaps(const AABB& other) const;

    [[nodiscard]] bool contains(const linalg::aliases::float3& pt) const;

    // entirely contains and surrounds the other
    [[nodiscard]] bool contains(const AABB& other) const;

    // expand AABB to include a point
    void expand(const linalg::aliases::float3& point);

    [[nodiscard]] std::array<linalg::aliases::float3, 8> getCorners() const;

    // efficient AABB computation for oriented boxes.
    // computes the new AABB that is large enough to contain the oriented AABB,
    // and moves it to new displacement. this is an information losing
    // transform!!
    [[nodiscard]] AABB computeMovedAndScaledAABB(
      const linalg::aliases::float3& displacement,
      const linalg::aliases::float3x3& rotationMatrix) const;
};
