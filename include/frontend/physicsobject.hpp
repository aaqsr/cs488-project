#pragma once

#include "model.hpp"
#include "util/quaternion.hpp"
#include "window.hpp"

#include "linalg.h"

class PhysicsObj
{
    Model &model;
    linalg::aliases::float3 prevPos = {0.0F, 0.0F, 0.0F};
    linalg::aliases::float3 constantForces = {0.0F, 0.0F, 0.0F};
    linalg::aliases::float3 initVelocity = {0.0F, 0.0F, 0.0F};
    float mass = 1.0f;
    Quaternion orientation{0.0F, 0.0F, 0.0F, 1.0F};

    // cached for performance for things that need it, hence mutable
    // (TODO: does this make a difference?)
    mutable bool vectorsNeedUpdate = true;

  public:
    PhysicsObj(Model& m);

    void update(float deltaTime);
    // void addVelocity(linalg::aliases::float3 velocity);

    void setPosition(const linalg::aliases::float3& pos);
    void move(const linalg::aliases::float3& displacement);
    void setOrientation(const Quaternion& quat);
    void rotate(const Quaternion& deltaRotation);
    void rotateAroundAxis(const linalg::aliases::float3& axis,
                          float angleRadians);

    [[nodiscard]] const linalg::aliases::float3& position() const
    {
        return model.worldPos;
    }
    [[nodiscard]] linalg::aliases::float3& position()
    {
        return model.worldPos;
    }
    [[nodiscard]] const Quaternion& getOrientation() const
    {
        return orientation;
    }
    [[nodiscard]] Model& getModel()
    {
        return model;
    }
    // [[nodiscard]] const linalg::aliases::float3& getForce() const
    // {
    //     return forces;
    // }
    [[nodiscard]] linalg::aliases::float3& getConstForce()
    {
        return constantForces;
    }
    [[nodiscard]] linalg::aliases::float3& getInitVelocity()
    {
        return initVelocity;
    }

};
