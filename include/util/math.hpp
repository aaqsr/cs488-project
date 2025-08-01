#pragma once

#include "linalg.h"
#include <cmath>
#include <limits>

class CS488Math
{

    // Fast random number generator based on xoshiro128+. NOT
    // cryptographically secure. Source: https://arxiv.org/abs/1805.01407
    class xoshiro128
    {
        // must be not all zero
        static inline uint32_t seed[4] = {0xDCDDC9DC, 0xE76A5E0B, 0x57D32C3E,
                                          0x1731DBEB};

        static uint32_t rotl(const uint32_t x, int k)
        {
            return (x << k) | (x >> (32 - k));
        }
        static uint32_t next(uint32_t* s)
        {
            const uint32_t result = s[0] + s[3];
            const uint32_t t = s[1] << 9;
            s[2] ^= s[0];
            s[3] ^= s[1];
            s[1] ^= s[2];
            s[0] ^= s[3];
            s[2] ^= t;
            s[3] = rotl(s[3], 11);
            return result;
        }
        static float randomToFloat(uint32_t random)
        {
            // grab 23 bits for the mantissa within [0,1)
            uint32_t u32 = (random >> 9) | 0x3f800000;
            float f = *reinterpret_cast<float*>(&u32);
            return f - 1.0F;
        }

      public:
        static float getRand()
        {
            return randomToFloat(next(seed));
        }
    };

    // A constexpr fast sqrt function.
    // Source: Taken from https://stackoverflow.com/a/34134071
    class sqrtContexpr
    {
        constexpr static float sqrtNewtonRaphson(float x, float curr,
                                                 float prev)
        {
            return curr == prev
                     ? curr
                     : sqrtNewtonRaphson(x, 0.5F * (curr + x / curr), curr);
        }

      public:
        constexpr static float sqrt(float x)
        {
            return x >= 0 && x < std::numeric_limits<float>::infinity()
                     ? sqrtNewtonRaphson(x, x, 0)
                     : std::numeric_limits<float>::quiet_NaN();
        }
    };

  public:
    constexpr static float sqrt(float x)
    {
        return sqrtContexpr::sqrt(x);
    }
    static float rand()
    {
        return xoshiro128::getRand();
    }
};
