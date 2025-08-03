#pragma once

#include "physics/spatialGrid.hpp"
#include "sim/waterSimulation.hpp"

// Forward declared
class RigidBodyData;

/**
 * @class SuperCollider
 * @brief Manages collision detection and resolution for the physics simulation.
 * <blockquote>
 *  "Super collider...
 *      I am dust in a mo-ment"
 * </blockquote> - Radiohead, Supercollider (2011)
 * @ingroup physics
 *
 * @details This class centralises all logic related to collisions. It follows a
 * standard multi-phase pipeline to efficiently handle interactions between a
 * large number of objects.
 *
 * @section Technicality
 * The collision pipeline is structured as follows:
 * 1.  **Broad Phase:** A `SpatialGrid` is used to quickly identify pairs of
 * objects whose AABBs are near each other. This avoids the O(n^2) cost of
 * checking every object against every other object.
 * 2.  **Narrow Phase:** For each potentially colliding pair identified in the
 * broad phase, a more precise AABB-AABB intersection test is performed via
 * `detectAABBCollision`.
 * 3.  **Resolution:** If a collision is confirmed, an impulse-based resolution
 * is applied in `resolveAABBCollision`. This involves calculating and applying
 * an impulse to the bodies' linear and angular momenta. Positional correction
 * is also applied to resolve interpenetration.
 *
 * @section Data and Code Sources
 * The impulse-based resolution method is a classic technique in game physics,
 * We drew some inspiration from the "Physics - Collision Detection" tutorial by
 * the University of Newcastle, Game Engineering Lab. Accessible at
 * https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/physicstutorials/4collisiondetection/
 *
 * @section Caveats
 * The current implementation only performs AABB-AABB collision detection. For
 * non-convex or more complex shapes, this can be inaccurate. A more advanced
 * engine would follow this phase with a precise narrow-phase algorithm like the
 * Separating Axis Theorem (SAT) on the actual mesh geometry. We attempted to
 * implement such a system many times. We could not get it to work each time.
 */
class SuperCollider
{
    /**
     * @name Collision Response Parameters
     * @brief These constants control the physical behaviour of collisions.
     * @{
     */

    /**
     * @brief The coefficient of restitution for AABB-AABB collisions. Controls
     * "bounciness". A value of 0.0 is perfectly inelastic (no bounce), 1.0 is
     * perfectly elastic.
     */
    constexpr static float aabbRestitutionCoefficient = 0.4F;

    /**
     * @brief An additional damping factor applied to angular velocity during an
     * AABB-AABB collision to increase stability and reduce excessive spinning
     * post-impact.
     */
    constexpr static float aabbCollisionAngularDamping = 0.85F;

    /**
     * @brief A factor controlling how aggressively interpenetration is
     * resolved. A higher value pushes objects apart more forcefully.
     */
    constexpr static float positionCorrectionStrength = 1.0F;

    /**
     * @brief A damping factor applied to the relative velocity at the contact
     * point during a collision. This helps to reduce jitter and increase
     * stability.
     */
    constexpr static float velocityDampingOnCollision = 0.95F;

    /** @} */

    /**
     * @brief The spatial grid used for broad-phase collision detection.
     * @details This static instance is shared across all collision processing
     * calls.
     */
    static inline SpatialGrid spatialGrid{};

    /**
     * @struct AABBCollisionInfo
     * @brief Stores the results of an inter-body AABB collision query.
     * @ingroup physics
     * @details This struct extends `CollisionInfo` by also storing the indices
     * of the two bodies involved in the collision.
     */
    struct AABBCollisionInfo
    {
        /** @brief Flag indicating if a collision occurred. */
        bool hasCollision = false;
        /** @brief The normal of the collision surface, pointing away from body
         * B. */
        linalg::aliases::float3 normal{0.0F};
        /** @brief The point of contact in world space. */
        linalg::aliases::float3 contactPoint{0.0F};
        /** @brief The distance of interpenetration along the collision normal.
         */
        float penetrationDepth = 0.0F;
        /** @brief The index of the first body in the main physics array. */
        size_t bodyAIndex = 0;
        /** @brief The index of the second body in the main physics array. */
        size_t bodyBIndex = 0;
    };

    /**
     * @brief Detects collision between two rigid bodies using their AABBs.
     * @param bodyA The first rigid body.
     * @param bodyB The second rigid body.
     * @param indexA The array index of the first body.
     * @param indexB The array index of the second body.
     * @return An `AABBCollisionInfo` struct with details of the collision.
     */
    static AABBCollisionInfo detectAABBCollision(const RigidBodyData& bodyA,
                                                 const RigidBodyData& bodyB,
                                                 size_t indexA, size_t indexB);

    /**
     * @brief Resolves a detected collision between two rigid bodies.
     * @details Applies an impulse to change the bodies' momenta and corrects
     * their positions to resolve penetration.
     * @param bodyA The first rigid body.
     * @param bodyB The second rigid body.
     * @param collision The details of the detected collision, as an
     * `AABBCollisionInfo` struct.
     */
    static void resolveAABBCollision(RigidBodyData& bodyA, RigidBodyData& bodyB,
                                     const AABBCollisionInfo& collision);

    /**
     * @struct CollisionInfo
     * @brief Stores the results of a collision query between a body and a
     * static boundary.
     * @ingroup physics
     * @details This is a simpler version of collision data used for resolving
     * interactions with the static world AABBs, where only one dynamic body is
     * involved.
     */
    struct CollisionInfo
    {
        /** @brief Flag indicating if a collision occurred. */
        bool hasCollision = false;
        /** @brief The normal of the collision surface, pointing away from the
         * boundary. */
        linalg::aliases::float3 normal{0.0F};
        /** @brief The point of contact in world space. */
        linalg::aliases::float3 contactPoint{0.0F};
        /** @brief The distance of interpenetration along the collision normal.
         */
        float penetrationDepth = 0.0F;
    };

    /**
     * @brief The coefficient of restitution for boundary collisions.
     */
    constexpr static float restitutionCoefficient = 0.6F;

    /**
     * @brief The angular damping factor for boundary collisions.
     */
    constexpr static float collisionAngularDamping = 0.95F;

    /**
     * @brief Detects collision between a rigid body and a static, world-aligned
     * AABB.
     * @param aabb The static boundary AABB to test against.
     * @param rigidBody The rigid body to test.
     * @return A `CollisionInfo` struct containing the details of the collision,
     * if any.
     */
    static CollisionInfo detectAABBCollision(const AABB& aabb,
                                             const RigidBodyData& rigidBody);

  public:
    /**
     * @name World Boundaries
     * @brief Static AABBs defining the simulation area.
     * @{
     */

    /**
     * @brief The height of the water surface in the pool.
     */
    constexpr static float poolHeight = 1.5F;

    /**
     * @brief An AABB defining the physical boundaries of the water pool.
     */
    const static inline AABB thePoolLimits{
      linalg::aliases::float3{WaterSimulation::bottomLeftCornerWorldPos_xz.x,
                              0.25F, WaterSimulation::bottomLeftCornerWorldPos_xz.y},
      linalg::aliases::float3{  WaterSimulation::topRightCornerWorldPos_xz.x,
                              2.0F,   WaterSimulation::topRightCornerWorldPos_xz.y }
    };
    /**
     * @brief An AABB defining the absolute outer limits of the simulation
     * world. Objects outside this volume may be disabled or removed.
     */
    const static inline AABB theWorldLimits{
      thePoolLimits.min - linalg::aliases::float3{5.0F, 3.0F, 5.0F},
      thePoolLimits.max + linalg::aliases::float3{5.0F, 8.0F, 5.0F}
    };

    /** @} */

    /**
     * @brief Processes all potential collisions between rigid bodies for a
     * single frame.
     * @param rigidBodies A vector of all dynamic rigid bodies in the
     * simulation.
     */
    static void processAABBCollisions(std::vector<RigidBodyData>& rigidBodies);

    /**
     * @brief Enforces boundary constraints on a rigid body, preventing it from
     * leaving the given AABB.
     * @param aabb The boundary AABB.
     * @param rigidBody The rigid body to constrain.
     */
    static void keepWithinAABB(AABB aabb, RigidBodyData& rigidBody);
};
