#pragma once

#include "model.hpp"
#include "util/quaternion.hpp"
#include "window.hpp"

#include "linalg.h"

class PhysicsObj
{
    Model &model;
    linalg::aliases::float3 prevPos = {0.0F, 0.0F, 0.0F};
    linalg::aliases::float3 com = {0.0F, 0.0F, 0.0F};
    linalg::aliases::float3 constantForces = {0.0F, 0.0F, 0.0F};
    linalg::aliases::float3 angularMomentum = {0.0F, 0.0F, 0.0F};
    linalg::aliases::float3x3 initInvInertia;
    linalg::aliases::float3 impulse{0.0f};
    linalg::aliases::float3 torque{0.0f};
    float mass = 1.0f;

    void calcInertia();

  public:
    PhysicsObj(Model& m);

    void update(float deltaTime);
    // void addVelocity(linalg::aliases::float3 velocity);
    void applyForce(linalg::aliases::float3 force, linalg::aliases::float3 contact);

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
        return model.rotation;
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
        return impulse;
    }

};
