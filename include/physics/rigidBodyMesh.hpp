#pragma once

#include <linalg.h>

// Forward declaration
struct SubTriangle;

/**
 * @struct Triangle
 * @ingroup physics
 *
 * @brief Represents a single, world-space triangle of a rigid body's physics
 * mesh.
 *
 * @details This structure holds the geometric data for a triangle, including
 * its vertices, normal, centroid, and area. It is used as the fundamental
 * primitive for physics interactions, particularly for calculating forces from
 * a fluid simulation. Not to be confused with `frontend/mesh.hpp`!
 * This is world-space CPU triangles that represent the rigid body so that we
 * can do physics on them. (In this case we primarily use them to compute forces
 * on the body as a result of the fluid simulation and vice versa.)
 */
struct Triangle
{
    /**
     * @brief The three vertices of the triangle in world coordinates.
     */
    linalg::aliases::float3 vertices[3];

    /**
     * @brief The geometric normal of the triangle, assumed to be pointing
     * outwards.
     */
    linalg::aliases::float3 normal;

    /**
     * @brief The geometric centre (barycentre) of the triangle.
     */
    linalg::aliases::float3 centroid;

    /**
     * @brief The surface area of the triangle.
     */
    float area;

    /**
     * @brief Constructs a Triangle and computes its geometric properties.
     * @details The normal is calculated via the cross product of two edge
     * vectors. The area is half the magnitude of this cross product. The
     * centroid is the arithmetic mean of the vertices.
     * @param v0 The first vertex.
     * @param v1 The second vertex.
     * @param v2 The third vertex.
     */
    Triangle(const linalg::aliases::float3& v0,
             const linalg::aliases::float3& v1,
             const linalg::aliases::float3& v2);

    /**
     * @brief Subdivides the triangle into a grid of smaller, approximately
     * equal-area sub-triangles.
     *
     * @details This method is used to increase the resolution of force
     * application, for example, when simulating fluid pressure. Instead of
     * applying one force at the main centroid, forces can be sampled and
     * applied at the centroids of many smaller sub-triangles, leading to more
     * stable and accurate rotational effects.
     *
     * @section Technicality
     * The subdivision is achieved by iterating over a grid in the triangle's 2D
     * parametric space, defined by barycentric coordinates. For each grid cell,
     * it generates a `SubTriangle`. The complexity is O(S^2), where S is the
     * number of subdivisions along an edge.
     *
     * @param subdivisions The number of divisions along each edge of the
     * triangle. A value of `N` results in `N*N` sub-triangles.
     * @return A vector of `SubTriangle` objects.
     */
    [[nodiscard]] std::vector<SubTriangle> subdivide(int subdivisions) const;
};

/**
 * @struct SubTriangle
 * @ingroup physics
 *
 * @brief A lightweight representation of a subdivided triangle patch.
 *
 * @details This struct is a performance optimization. When a `Triangle` is
 * subdivided, it produces many smaller patches. Storing full vertex data for
 * each would be inefficient. Instead, this struct holds only the essential data
 * needed for force calculations: the centroid (contact point), the normal, and
 * the area of the patch.
 */
struct SubTriangle
{
    /**
     * @brief The centroid of the sub-triangle patch in world space.
     */
    linalg::aliases::float3 centroid;

    /**
     * @brief The normal of the parent triangle.
     */
    linalg::aliases::float3 normal;

    /**
     * @brief The area of the sub-triangle patch.
     */
    float area;

    /**
     * @brief Constructs a SubTriangle.
     * @param cent The centroid of the patch.
     * @param norm The normal of the patch.
     * @param a The area of the patch.
     */
    SubTriangle(const linalg::aliases::float3& cent,
                const linalg::aliases::float3& norm, float a);
};
