#pragma once

#include "physics/constants.hpp"
#include "util/quaternion.hpp"

#include <linalg.h>

class RigidBody
{
    // Computation is done there for...reasons
    friend class PhysicsEngine;

    // Linear motion
    float mass = 1.0F;

    linalg::aliases::float3 constantAcceleration =
      linalg::aliases::float3{0.0F, 0.0F, 0.0F} +
      Physics::gravitationalAcceleration;

    linalg::aliases::float3 globalCentreOfMassPos = {0.0F, 0.0F, 0.0F};
    linalg::aliases::float3 prevGlobalCentreOfMassPos = {0.0F, 0.0F, 0.0F};

    // Rotational motion
    Quaternion orientation{0.0F, 0.0F, 0.0F, 1.0F};
    Quaternion prevOrientation{0.0F, 0.0F, 0.0F, 1.0F};
    linalg::aliases::float3x3 inertiaTensor{linalg::identity}; // local space
    linalg::aliases::float3x3 invInertiaTensor{linalg::identity};

    // Acummulated forces (for one time step)
    linalg::aliases::float3 accumulatedForce = {0.0F, 0.0F, 0.0F};
    linalg::aliases::float3 accumulatedTorque = {0.0F, 0.0F, 0.0F};

    [[nodiscard]] linalg::aliases::float3 getTotalAcceleration() const;

    void setConstForce(const linalg::aliases::float3& force);

    void clearAccumulatedForceAndTorque();

  public:
    RigidBody(const linalg::aliases::float3& initPos = {0.0F, 0.0F, 0.0F},
              const linalg::aliases::float3& initVel = {0.0F, 0.0F, 0.0F});

    [[nodiscard]] const linalg::aliases::float3& getCentreOfMass() const;

    void accumulateForceForNextTimeStep(const linalg::aliases::float3& force);
    void accumulateTorqueForNextTimeStep(const linalg::aliases::float3& torque);
};
