#include "frontend/physicsobject.hpp"
#include "linalg.h"
#include <numbers>

namespace
{
linalg::aliases::float3 globalGravity = {0.0F, -9.8F, 0.0F};

} // namespace

PhysicsObj::PhysicsObj(Model& m): model{m}, prevPos{m.worldPos}, orientation{Quaternion::identity()}
// init with identity quaternion
{}

void PhysicsObj::update(float deltaTime) {
    linalg::aliases::float3 temp = position();
    if (initVelocity.x != 0 || initVelocity.y != 0 || initVelocity.z != 0) {
        printf("WOOOOOO\n");
        position() += initVelocity * deltaTime;
        initVelocity = {0.0f, 0.0f, 0.0f};
    } else {
        position() = 2*position() - prevPos + deltaTime*deltaTime*(globalGravity + constantForces/mass + initVelocity/mass);
    }
    prevPos = temp;
}

// void PhysicsObj::addVelocity(linalg::aliases::float3 velocity) {
//     prevPos = position();
// 	position() += velocity * 0.02f;
// }

void PhysicsObj::setPosition(const linalg::aliases::float3& pos)
{
    position() = pos;
    prevPos = pos;
}

void PhysicsObj::move(const linalg::aliases::float3& displacement)
{
    prevPos = position();
    position() = position() + displacement;
}

void PhysicsObj::setOrientation(const Quaternion& quat)
{
    orientation = quat.normalized();
    vectorsNeedUpdate = true;
}

void PhysicsObj::rotate(const Quaternion& deltaRotation)
{
    orientation = (deltaRotation * orientation).normalized();
    vectorsNeedUpdate = true;
}

void PhysicsObj::rotateAroundAxis(const linalg::aliases::float3& axis,
                              float angleRadians)
{
    const Quaternion axisRotation =
      Quaternion::fromAxisAngle(axis, angleRadians);
    rotate(axisRotation);
}
