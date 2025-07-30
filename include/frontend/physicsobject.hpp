#pragma once

#include "model.hpp"
#include "util/quaternion.hpp"
#include "window.hpp"

#include "linalg.h"

class Inertia {
    float int_x, int_x2, int_y2, int_z2, int_x3, int_y3, int_z3, int_x2y, int_y2z, int_z2x;
    static void calcTemps(const linalg::aliases::float3& w, float& f1, float& f2, float& f3);
    void calcIntegrals(const linalg::aliases::float3& x, const linalg::aliases::float3& y, const linalg::aliases::float3& z);
  public:
    float inertia_xx, inertia_yy, inertia_zz, inertia_xy, inertia_yz, inertia_zx;
    float mass = 0.0f, volume = 0.0f;
    linalg::aliases::float3 com{0.0f};

    Inertia() {
        inertia_xx = inertia_yy = inertia_zz = inertia_xy = inertia_yz = inertia_zx = 0.0f;
    }

    void add_face(const linalg::aliases::float3& v0, const linalg::aliases::float3& v1, const linalg::aliases::float3& v2, float density);
    void correct_inertia_with_com();

    void rescale(float scale);

    linalg::aliases::float3x3 initialInertiaMatrix();

};

class PhysicsObj
{
    Model &model;
    linalg::aliases::float3 prevPos = {0.0F, 0.0F, 0.0F};
    linalg::aliases::float3 com = {0.0F, 0.0F, 0.0F};
    linalg::aliases::float3 angularMomentum = {0.0F, 0.0F, 0.0F};
    linalg::aliases::float3 constantForces = {0.0F, 0.0F, 0.0F};

    linalg::aliases::float3 impulse{0.0f};
    linalg::aliases::float3 torque{0.0f};

    Inertia inertia;
    linalg::aliases::float3x3 initInvInertia;
    float mass;

    float density = 1.0f;

    void calcInertia(linalg::aliases::float3 scale = {1.0f, 1.0f, 1.0f});

  public:
    PhysicsObj(Model& m): model{m}, prevPos{m.worldPos} {
        model.rotation = Quaternion::identity();
        calcInertia();
    };

    void update(float deltaTime);
    // void addVelocity(linalg::aliases::float3 velocity);
    void applyForce(linalg::aliases::float3 force, linalg::aliases::float3 contact);
    void rescale(const linalg::aliases::float3& scale = {1.0f, 1.0f, 1.0f});
    void rescale(const float& scale);

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
