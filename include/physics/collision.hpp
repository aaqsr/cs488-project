#pragma once

#include "physics/spatialGrid.hpp"
#include "sim/waterSimulation.hpp"

class RigidBodyData;

// Collision detector and resolver
// "Super collider...
//        ... I am dust in a moooment"
class SuperCollider
{
    constexpr static float aabbRestitutionCoefficient = 0.4F;
    constexpr static float aabbCollisionAngularDamping = 0.85F;
    constexpr static float positionCorrectionStrength = 1.0F;
    constexpr static float velocityDampingOnCollision = 0.95F;

    static inline SpatialGrid spatialGrid{};

    struct AABBCollisionInfo
    {
        bool hasCollision = false;
        linalg::aliases::float3 normal{0.0F};
        linalg::aliases::float3 contactPoint{0.0F};
        float penetrationDepth = 0.0F;
        size_t bodyAIndex = 0;
        size_t bodyBIndex = 0;
    };

    static AABBCollisionInfo detectAABBCollision(const RigidBodyData& bodyA,
                                                 const RigidBodyData& bodyB,
                                                 size_t indexA, size_t indexB);

    static void resolveAABBCollision(RigidBodyData& bodyA, RigidBodyData& bodyB,
                                     const AABBCollisionInfo& collision);

    struct CollisionInfo
    {
        bool hasCollision = false;
        linalg::aliases::float3 normal{0.0F};
        linalg::aliases::float3 contactPoint{0.0F};
        float penetrationDepth = 0.0F;
    };
    constexpr static float restitutionCoefficient = 0.6F;
    constexpr static float collisionAngularDamping = 0.95F;
    static CollisionInfo detectAABBCollision(const AABB& aabb,
                                             const RigidBodyData& rigidBody);

  public:
    constexpr static float poolHeight = 1.5F;
    const static inline AABB thePoolLimits{
      linalg::aliases::float3{WaterSimulation::bottomLeftCornerWorldPos_xz.x,
                              0.25F, WaterSimulation::bottomLeftCornerWorldPos_xz.y},
      linalg::aliases::float3{  WaterSimulation::topRightCornerWorldPos_xz.x,
                              2.0F,   WaterSimulation::topRightCornerWorldPos_xz.y }
    };
    const static inline AABB theWorldLimits{
      thePoolLimits.min - linalg::aliases::float3{5.0F, 3.0F, 5.0F},
      thePoolLimits.max + linalg::aliases::float3{5.0F, 8.0F, 5.0F}
    };

    static void processAABBCollisions(std::vector<RigidBodyData>& rigidBodies);
    static void keepWithinAABB(AABB aabb, RigidBodyData& rigidBody);
};
