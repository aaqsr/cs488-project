#pragma once

#include <algorithm>
#include <array>
#include <linalg.h>

/**
 * @struct AABB
 * @brief Defines an Axis-Aligned Bounding Box (AABB).
 * @ingroup physics
 *
 * @details An AABB is a rectangular cuboid whose faces are parallel to the
 * coordinate planes. It is defined by two opposing corner vertices: the minimum
 * (`min`) and maximum (`max`) extents along each coordinate axis.
 *
 * @section Technicality
 * AABBs are a cornerstone of efficient "broad-phase" collision detection. The
 * simplicity of their geometry allows for extremely fast overlap tests, which
 * are used to quickly cull pairs of objects that are not close enough to
 * warrant more expensive, precise collision checks.
 *
 * @section Performance
 * All fundamental operations on this struct (overlap, expand, contains) are
 * O(1) and involve only a handful of floating-point comparisons, making them
 * exceptionally performant.
 */
struct AABB
{
    /**
     * @brief The vertex of the AABB with the minimum coordinates on all axes.
     */
    linalg::aliases::float3 min;

    /**
     * @brief The vertex of the AABB with the maximum coordinates on all axes.
     */
    linalg::aliases::float3 max;

    /**
     * @brief Default constructor. Initializes a zero-volume AABB at the origin.
     */
    AABB();

    /**
     * @brief Constructs an AABB with specified minimum and maximum corners.
     * @param min The minimum corner of the box.
     * @param max The maximum corner of the box.
     */
    AABB(const linalg::aliases::float3& min,
         const linalg::aliases::float3& max);

    /**
     * @brief Checks for intersection with another AABB.
     * @details Two AABBs overlap if and only if their projection intervals
     * overlap on all three coordinate axes. This is a direct application of the
     * Separating Axis Theorem (SAT), simplified for the axis-aligned case.
     * @param other The other AABB to test against.
     * @return `true` if the boxes have a non-empty intersection, `false`
     * otherwise.
     */
    [[nodiscard]] bool overlaps(const AABB& other) const;

    /**
     * @brief Checks if a point is contained within the AABB.
     * @param pt The point to test.
     * @return `true` if the point is inside or on the boundary of the box,
     * `false` otherwise.
     */
    [[nodiscard]] bool contains(const linalg::aliases::float3& pt) const;

    /**
     * @brief Checks if this AABB completely encloses another AABB.
     * @param other The other AABB to test.
     * @return `true` if the `other` box is fully inside this one, `false`
     * otherwise.
     */
    [[nodiscard]] bool contains(const AABB& other) const;

    /**
     * @brief Expands the AABB to enclose a given point.
     * @details This is an idempotent operation used to construct an AABB from a
     * set of points (e.g., the vertices of a mesh). The method updates the
     * `min` and `max` extents as necessary to include the new point.
     * @param point The point to be included within the AABB.
     */
    void expand(const linalg::aliases::float3& point);

    /**
     * @brief Retrieves the eight corner vertices of the AABB.
     * @return A std::array containing the 8 corner points of the box.
     */
    [[nodiscard]] std::array<linalg::aliases::float3, 8> getCorners() const;

    /**
     * @brief Computes a new AABB that tightly encloses this AABB after a
     * rotation and translation.
     *
     * @details This method calculates the axis-aligned bounding box of an
     * *oriented* bounding box (OBB). It is an "information-losing"
     * transformation because the tightest possible fit for a rotated box is
     * generally not axis-aligned. The resulting AABB will be larger than the
     * OBB it contains.
     *
     * @section Technicality
     * The algorithm is a standard and highly efficient method for this
     * conversion, often referred to as "re-fitting" or "re-aligning" an AABB.
     * It is described in sources such as Christer Ericson's "Real-Time
     * Collision Detection". The new centre is found by transforming the
     * original centre. The new extents (half-dimensions) are calculated by
     * projecting the original extents onto the new axes. This projection is
     * achieved by taking the absolute values of the rotation matrix components
     * and multiplying them by the original extents. This correctly finds the
     * maximum reach of the rotated box along each world axis.
     *
     * @param displacement The translation to apply to the AABB's centre.
     * @param rotationMatrix The 3x3 rotation matrix to apply.
     * @return A new AABB that encloses the transformed original.
     */
    [[nodiscard]] AABB computeMovedAndScaledAABB(
      const linalg::aliases::float3& displacement,
      const linalg::aliases::float3x3& rotationMatrix) const;
};
