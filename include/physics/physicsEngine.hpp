#pragma once

#include "physics/AABB.hpp"
#include "physics/spatialGrid.hpp"
#include "sim/waterSimulation.hpp"
#include "util/channel.hpp"

#include <linalg.h>

#include <memory>
#include <vector>

class Model;
class RigidBodyCharacteristics;

// Can easily be converted into a command like object if we also need to be able
// to remove objects by command or something
struct PhysicsEngineReceiverData
{
    std::shared_ptr<Model> model;
    linalg::aliases::float3 scale{1.0F};
    linalg::aliases::float3 initPos{0.0F};
    linalg::aliases::float3 initVel{0.0F};
    linalg::aliases::float3 initAngVel{0.0F};

    linalg::aliases::float3 initForce{0.0F};
    linalg::aliases::float3 initForceContact{0.0F};

    float density = 1.0F;
};

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

  public:
    static void processAABBCollisions(std::vector<RigidBodyData>& rigidBodies);
};

class PhysicsEngine
{
    constexpr static float linearDamping = 1.0F;
    constexpr static float angularDamping = 1.0F;

    Receiver<std::vector<PhysicsEngineReceiverData>>& channel;

    std::vector<std::shared_ptr<RigidBodyCharacteristics>>
      rigidBodyCharacteristics;

    constexpr static float poolHeight = 1.5F;
    inline static const AABB thePoolLimits{
      linalg::aliases::float3{WaterSimulation::bottomLeftCornerWorldPos_xz.x,
                              0.25F, WaterSimulation::bottomLeftCornerWorldPos_xz.y},
      linalg::aliases::float3{  WaterSimulation::topRightCornerWorldPos_xz.x,
                              2.0F,   WaterSimulation::topRightCornerWorldPos_xz.y }
    };
    inline static const AABB theWorldLimits{
      thePoolLimits.min - linalg::aliases::float3{5.0F, 3.0F, 5.0F},
      thePoolLimits.max + linalg::aliases::float3{5.0F, 8.0F, 5.0F}
    };
    static void keepWithinAABB(AABB aabb, RigidBodyData& rigidBody);

    void simulateRigidBody(RigidBodyData& out, const RigidBodyData& prev);

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
    PhysicsEngine(Receiver<std::vector<PhysicsEngineReceiverData>>& recv);
    void updateRigidBodies(std::vector<RigidBodyData>& rigidBodies,
                           const std::vector<RigidBodyData>& prevRigidBodies);
};
