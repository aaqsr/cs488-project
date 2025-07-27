#pragma once

#include "physics/AABB.hpp"
#include "physics/constants.hpp"
#include "physics/inertiaTensor.hpp"
#include "util/error.hpp"
#include "util/quaternion.hpp"

#include <linalg.h>
#include <memory>

class Model;
struct Triangle;

// For static data that does not vary between frames
class RigidBodyCharacteristics
{
    std::shared_ptr<Model> model;

    float density = 1.0F;
    linalg::aliases::float3 scale{1.0F};

    Inertia inertiaTensor;
    linalg::aliases::float3x3 initInvInertia{linalg::identity};

    // linalg::aliases::float3 constantAcceleration =
    //   linalg::aliases::float3{0.0F, 0.0F, 0.0F} +
    //   Physics::gravitationalAcceleration;

  public:
    RigidBodyCharacteristics(std::shared_ptr<Model> rigidBodyModel,
                             linalg::aliases::float3 scale, float density);

    RigidBodyCharacteristics(const RigidBodyCharacteristics&) = delete;
    RigidBodyCharacteristics(RigidBodyCharacteristics&&) = default;
    RigidBodyCharacteristics&
    operator=(const RigidBodyCharacteristics&) = delete;
    RigidBodyCharacteristics& operator=(RigidBodyCharacteristics&&) = default;
    ~RigidBodyCharacteristics() = default;

    [[nodiscard]] const Inertia& getInertiaTensor() const;
    [[nodiscard]] const linalg::aliases::float3x3& getInitInvInertia() const;
    [[nodiscard]] std::shared_ptr<Model> getModelPtr() const;
    [[nodiscard]] const linalg::aliases::float3& getScale() const;
};

// For data that may vary every frame
class RigidBodyData
{
    friend class PhysicsEngine;

    std::shared_ptr<RigidBodyCharacteristics> characteristics;

    // Linear motion
    linalg::aliases::float3 worldPosition;
    linalg::aliases::float3 prevWorldPosition;

    // Acummulated forces (for one time step)
    linalg::aliases::float3 accumulatedForce = {0.0F, 0.0F, 0.0F};
    linalg::aliases::float3 accumulatedTorque = {0.0F, 0.0F, 0.0F};

    // Rotational motion
    linalg::aliases::float3 angularMomentum = {0.0F, 0.0F, 0.0F};
    Quaternion orientation{0.0F, 0.0F, 0.0F, 1.0F};

    mutable std::vector<Triangle> cachedTriangles;
    mutable bool trianglesCacheValid = false;

  public:
    RigidBodyData(std::shared_ptr<RigidBodyCharacteristics> characteristics,
                  const linalg::aliases::float3& initPos = {0.0F, 0.0F, 0.0F},
                  const linalg::aliases::float3& initVel = {0.0F, 0.0F, 0.0F},
                  const linalg::aliases::float3& initAngVel = {0.0F, 0.0F,
                                                               0.0F});

    RigidBodyData(const RigidBodyData&) = default;
    RigidBodyData(RigidBodyData&&) = default;
    RigidBodyData& operator=(const RigidBodyData&) = delete;
    RigidBodyData& operator=(RigidBodyData&&) = delete;
    ~RigidBodyData() = default;

    void applyForce(linalg::aliases::float3 force,
                    linalg::aliases::float3 contact);

    [[nodiscard]] std::shared_ptr<Model> getModelPtr() const;

    [[nodiscard]] const linalg::aliases::float3& getWorldPosition() const;
    [[nodiscard]] const linalg::aliases::float3& getScale() const;
    [[nodiscard]] const Quaternion& getOrientation() const;

    [[nodiscard]] linalg::aliases::float3 getLinearVelocity() const;
    [[nodiscard]] linalg::aliases::float3 getAngularVelocity() const;
    [[nodiscard]] linalg::aliases::float3 getWorldPosOfCenterOfMass() const;

    // velocity of any point on the rigid body
    [[nodiscard]] linalg::aliases::float3
    getPointVelocity(const linalg::aliases::float3& point) const;

    // extract trianlges out of the mesh
    [[nodiscard]] const std::vector<Triangle>& getTriangles() const;
    void invalidateTriangleCache();

    AABB computeAABB() const;
};
