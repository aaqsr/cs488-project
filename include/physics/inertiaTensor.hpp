#pragma once

#include <linalg.h>

class Mesh;

/**
 * @class Inertia
 * @ingroup physics
 *
 * @brief Computes the inertial properties of a rigid body from its polygonal
 * mesh representation.
 *
 * @details This class calculates the mass, volume, centre of mass (COM), and
 * the inertia tensor for a 3D model defined by a closed, manifold triangle
 * mesh. The physical properties are derived assuming the object has a uniform
 * density.
 *
 * @section Technicality
 * The underlying algorithm is a direct implementation of the method described
 * by Brian Mirtich in his 1996 paper, "Fast and Accurate Computation of
 * Polyhedral Mass Properties." This technique is notable for its accuracy and
 * efficiency. It leverages the divergence theorem to convert volume integrals,
 * which are computationally expensive, into surface integrals over the object's
 * boundary triangles. These surface integrals are then further simplified into
 * line integrals around the triangle edges.
 *
 * The algorithm computes integrals over a set of tetrahedra, where each
 * tetrahedron is formed by a mesh triangle and the coordinate origin. By
 * summing the signed contributions of these tetrahedra, the properties of the
 * complete polyhedron are determined.
 *
 * @section Assumptions and Caveats
 * - **Closed Mesh:** The input `bodyMeshes` must form a closed, "watertight"
 * surface. Holes or gaps in the mesh will lead to incorrect volume and mass
 * calculations.
 * - **Manifold Geometry:** The mesh should be manifold (i.e., every edge is
 * shared by exactly two faces).
 * - **Winding Order:** The vertices of each face must have a consistent winding
 * order (e.g., counter-clockwise) such that the face normals consistently point
 * outwards from the volume. Inconsistent normals will cause volumes to be
 * incorrectly added or subtracted.
 * - **Numerical Stability:** The method can suffer from numerical precision
 * issues if the object's coordinates are very large. For best results, the mesh
 * should be centered around the origin before calculations begin, though this
 * is not enforced by the class itself. The `cleanupNumericalNoise` method is a
 * post-process step to mitigate some of these effects.
 *
 * @section Performance
 * The computational complexity is linear, O(N), where N is the total number of
 * faces in the input meshes. This is highly efficient as it avoids volumetric
 * discretisation (voxelization) and requires only a single pass over the
 * geometry data.
 */
class Inertia
{
    /**
     * @brief The mass of the body, calculated as volume * density.
     */
    float mass = 0.0F;

    /**
     * @brief The volume of the mesh, calculated by summing the signed volumes
     * of tetrahedra.
     */
    float volume = 0.0F;

    /**
     * @brief The computed centre of mass (COM) of the body, in its local
     * coordinate frame.
     */
    linalg::aliases::float3 com{0.0F};

    /**
     * @name Volume Integrals
     * @brief These member variables accumulate the results of the volume
     * integrals described by Mirtich. They represent projections of the body's
     * volume onto the coordinate planes and other related quantities. For
     * example, `int_x` is the integral of `x dV` over the body's volume. These
     * are intermediate values used to derive the final mass, COM, and inertia
     * tensor.
     * @{
     */
    float int_x{0.0F}, int_x2{0.0F}, int_y2{0.0F}, int_z2{0.0F}, int_x3{0.0F},
      int_y3{0.0F}, int_z3{0.0F}, int_x2y{0.0F}, int_y2z{0.0F}, int_z2x{0.0F};
    /** @} */

    /**
     * @name Inertia Tensor Components
     * @brief Components of the inertia tensor calculated with respect to the
     * coordinate system's origin. These values must be corrected using the
     * parallel axis theorem to be relative to the centre of mass. `inertia_xx`,
     * `inertia_yy`, `inertia_zz` are the moments of inertia. `inertia_xy`,
     * `inertia_yz`, `inertia_zx` are the products of inertia.
     * @{
     */
    float inertia_xx{0.0F}, inertia_yy{0.0F}, inertia_zz{0.0F},
      inertia_xy{0.0F}, inertia_yz{0.0F}, inertia_zx{0.0F};
    /** @} */

    /**
     * @brief Processes a single tetrahedron (formed by a triangle and the
     * origin) and accumulates its contribution to the total volume integrals.
     * @param x A vector containing the x-coordinates of the triangle's three
     * vertices.
     * @param y A vector containing the y-coordinates of the triangle's three
     * vertices.
     * @param z A vector containing the z-coordinates of the triangle's three
     * vertices.
     */
    void calcIntegrals(const linalg::aliases::float3& x,
                       const linalg::aliases::float3& y,
                       const linalg::aliases::float3& z);

    /**
     * @brief Adjusts the inertia tensor to be relative to the centre of mass.
     * @details This method applies the parallel axis theorem to translate the
     * moments and products of inertia from the coordinate origin to the body's
     * computed centre of mass. This is a necessary final step to obtain the
     * body-frame inertia tensor used in physical simulation.
     */
    void correctInertiaWithCOM();

    /**
     * @brief Zeroes out very small values in the inertia tensor and centre of
     * mass.
     * @details This function serves to correct for floating-point inaccuracies
     * that can arise during the integration process, especially for highly
     * symmetric objects where off-diagonal products of inertia should
     * theoretically be zero.
     * @param threshold A small value below which floating-point numbers are
     * considered to be zero. The default value may require tuning.
     */
    void cleanupNumericalNoise(float threshold = 5e-7);

  public:
    /**
     * @brief Constructs an Inertia object and computes the physical properties
     * of the given meshes.
     * @param bodyMeshes A vector of `Mesh` objects that collectively form the
     * rigid body's surface.
     * @param scale A uniform scaling factor applied to the mesh vertices before
     * computation.
     * @param density The uniform density of the material from which the body is
     * made.
     */
    Inertia(const std::vector<Mesh>& bodyMeshes,
            linalg::aliases::float3 scale = {1.0F, 1.0F, 1.0F},
            float density = 1.0F);

    /**
     * @brief Adds the contribution of a single triangular face to the total
     * inertial properties.
     * @details This is the core computational routine that is called for every
     * triangle in the mesh.
     * @param v0 The first vertex of the triangle.
     * @param v1 The second vertex of the triangle.
     * @param v2 The third vertex of the triangle.
     * @param density The density of the object.
     */
    void addFace(const linalg::aliases::float3& v0,
                 const linalg::aliases::float3& v1,
                 const linalg::aliases::float3& v2, float density);

    /**
     * @brief Finalises the computation after all faces have been added.
     * @details This method calculates the final mass, COM, and inertia tensor
     * from the accumulated volume integrals. It applies the density, corrects
     * the tensor using the parallel axis theorem, and cleans up numerical
     * noise.
     * @param density The uniform density of the material.
     */
    void finalise(float density);

    /**
     * @brief Applies a uniform scaling factor to the already computed inertial
     * properties.
     * @details This is more efficient than recomputing from the scaled mesh.
     * Mass and volume scale with the cube of the scale factor, whilst the
     * inertia tensor scales with the fifth power. This method assumes a uniform
     * scale.
     * @param scale The uniform scaling factor.
     */
    void rescale(float scale);

    /**
     * @brief Retrieves the computed mass of the body.
     * @return The total mass.
     */
    [[nodiscard]] float getMass() const;

    /**
     * @brief Retrieves the computed centre of mass.
     * @return A 3D vector representing the COM in the body's local coordinates.
     */
    [[nodiscard]] const linalg::aliases::float3& getCentreOfMass() const;

    /**
     * @brief Constructs and returns the final 3x3 inertia tensor matrix.
     * @details The returned matrix is relative to the body's centre of mass.
     * @return A 3x3 matrix representing the moments and products of inertia.
     */
    [[nodiscard]] linalg::aliases::float3x3 initialInertiaMatrix() const;
};
