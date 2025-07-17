#pragma once

#include "physics/constants.hpp"
#include "util/quaternion.hpp"

#include <linalg.h>

class PhysicsObj
{
    friend class PhysicsEngine;

    float mass = 1.0F;

    linalg::aliases::float3 constantAcceleration =
      linalg::aliases::float3{0.0F, 0.0F, 0.0F} +
      Physics::gravitationalAcceleration;

    linalg::aliases::float3 pos = {0.0F, 0.0F, 0.0F};
    linalg::aliases::float3 prevPos = {0.0F, 0.0F, 0.0F};
    Quaternion rotation{0.0F, 0.0F, 0.0F, 1.0F};

    void setConstForce(const linalg::aliases::float3& force);

  public:
    PhysicsObj(const linalg::aliases::float3& initPos = {0.0F, 0.0F, 0.0F},
               const linalg::aliases::float3& initVel = {0.0F, 0.0F, 0.0F});

    [[nodiscard]] const linalg::aliases::float3& getPosition() const;
    [[nodiscard]] const linalg::aliases::float3& getPrevPosition() const;

    [[nodiscard]] const linalg::aliases::float3& getConstAccel() const;
};

