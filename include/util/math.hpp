#pragma once

#include "linalg.h"
#include <cmath>
#include <cstdint>
#include <limits>

/**
 * @class CS488Math
 * @brief A collection of static mathematical utility functions.
 * @ingroup util
 */
class CS488Math
{
    /**
     * @class xoshiro128
     * @brief A fast, non-cryptographic pseudo-random number generator.
     * @ingroup util
     *
     * @section Technicality
     * This is an implementation of the `xoshiro128+` algorithm. It is part of
     * the xoshiro/xoroshiro family of generators known for their excellent
     * statistical properties, speed, and small state size. It is a significant
     * improvement over older generators like `rand()` or LCGs.
     *
     * The `randomToFloat` method uses a common technique to convert the integer
     * output into a floating-point number in the range [0, 1) by directly
     * manipulating the bits of the IEEE 754 representation.
     *
     * @section Data and Code Sources
     * The algorithm and its properties are described by David Blackman and
     * Sebastiano Vigna in their 2018 paper, "Scrambled Linear-Feedback Shift
     * Registers". The source code is adapted from their public domain reference
     * implementations.
     *
     * @section Caveats
     * This generator is **not cryptographically secure** and should
     * not be used for applications requiring true unpredictability.
     */
    class xoshiro128
    {
        /** @brief The start seed of the generator. Random values picked
         * randomly. Must not be all zeros.
         */
        static inline uint32_t seed[4] = {0xDCDDC9DC, 0xE76A5E0B, 0x57D32C3E,
                                          0x1731DBEB};

        /** @brief Performs a bitwise left rotation. */
        static uint32_t rotl(uint32_t x, int k)
        {
            return (x << k) | (x >> (32 - k));
        }
        /** @brief Generates the next 32-bit random integer. */
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
        /** @brief Converts a 32-bit integer to a float in the range [0, 1). */
        static float randomToFloat(uint32_t random)
        {
            // grab 23 bits for the mantissa within [0,1)
            uint32_t u32 = (random >> 9) | 0x3f800000;
            float f = *reinterpret_cast<float*>(&u32);
            return f - 1.0F;
        }

      public:
        /**
         * @brief Gets the next random float in the range [0, 1).
         * @return A pseudo-random float.
         */
        static float getRand()
        {
            return randomToFloat(next(seed));
        }
    };

    /**
     * @class sqrtContexpr
     * @brief A compile-time square root function for floating-point numbers.
     * @ingroup util
     *
     * @section Technicality
     * This class implements a `constexpr` square root using the
     * **Newton-Raphson method**, an iterative root-finding algorithm. By
     * implementing it as a recursive `constexpr` function, the compiler can
     * evaluate the square root of a literal value at compile time, embedding
     * the result directly into the executable and avoiding any runtime
     * calculation.
     *
     * @section Data and Code Sources
     * Taken from https://stackoverflow.com/a/34134071
     */
    class sqrtContexpr
    {
        /** @brief The recursive Newton-Raphson implementation. */
        constexpr static float sqrtNewtonRaphson(float x, float curr,
                                                 float prev)
        {
            return curr == prev
                     ? curr
                     : sqrtNewtonRaphson(x, 0.5F * (curr + x / curr), curr);
        }

      public:
        /**
         * @brief Calculates the square root of a number at compile time.
         * @param x The non-negative number to take the square root of.
         * @return The compile-time computed square root of x.
         */
        constexpr static float sqrt(float x)
        {
            return x >= 0 && x < std::numeric_limits<float>::infinity()
                     ? sqrtNewtonRaphson(x, x, 0)
                     : std::numeric_limits<float>::quiet_NaN();
        }
    };

  public:
    /**
     * @brief A public alias for the random number generator's main function.
     * @return A pseudo-random float in the range [0, 1).
     */
    static float rand()
    {
        return xoshiro128::getRand();
    }

    /**
     * @brief Returns a pseudo-random float in the given range [min, max).
     * @param min The minimum value of the range (inclusive).
     * @param max The maximum value of the range (exclusive).
     * @return A float in the range [min, max).
     */
    static float randInRange(float min, float max)
    {
        return min + (rand() * (max - min));
    }

    /**
     * @brief A public alias for the compile-time square root function.
     * @return The compile-time computed square root of x.
     */
    constexpr static float sqrt(float x)
    {
        return sqrtContexpr::sqrt(x);
    }
};
