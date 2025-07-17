#pragma once

#include "frontend/model.hpp"
#include "frontend/shader.hpp"
#include "util/quaternion.hpp"

#include "linalg.h"

class PhysicsObj
{
    Model model;

    float mass = 1.0F;

    linalg::aliases::float3 constantForces = {0.0F, 0.0F, 0.0F};

    linalg::aliases::float3 pos = {0.0F, 0.0F, 0.0F};
    linalg::aliases::float3 prevPos = {0.0F, 0.0F, 0.0F};
    Quaternion rotation{0.0F, 0.0F, 0.0F, 1.0F};

  public:
    PhysicsObj(Model m,
               const linalg::aliases::float3& initPos = {0.0F, 0.0F, 0.0F},
               const linalg::aliases::float3& initVel = {0.0F, 0.0F, 0.0F});

    void update();
    // void addVelocity(linalg::aliases::float3 velocity);

    void move(const linalg::aliases::float3& displacement);
    [[nodiscard]] const Quaternion& getRotation() const;
    // void setRotation(const Quaternion& quat);
    // void rotate(const Quaternion& deltaRotation);
    // void rotateAroundAxis(const linalg::aliases::float3& axis,
    //                       float angleRadians);

    [[nodiscard]] const linalg::aliases::float3& getPosition() const;

    [[nodiscard]] const linalg::aliases::float3& getConstForce() const;
    void setConstForce(const linalg::aliases::float3& force);

    void draw(Shader::BindObject& shader);
};
