#pragma once

#include "linalg.h"
#include <cmath>
#include <limits>

namespace CS488Math
{

namespace Private
{

constexpr float sqrtNewtonRaphson(float x, float curr, float prev)
{
    return curr == prev ? curr
                        : sqrtNewtonRaphson(x, 0.5F * (curr + x / curr), curr);
}

} // namespace Private

// A constexpr fast sqrt function.
// Source: Taken from https://stackoverflow.com/a/34134071
constexpr float sqrt(float x)
{
    return x >= 0 && x < std::numeric_limits<float>::infinity()
             ? Private::sqrtNewtonRaphson(x, x, 0)
             : std::numeric_limits<float>::quiet_NaN();
}

} // namespace CS488Math
