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

    [[nodiscard]] bool overlaps(const AABB& other) const;

    // expand AABB to include a point
    void expand(const linalg::aliases::float3& point);
};
