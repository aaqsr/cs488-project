#include "physics/physicsObj.hpp"
#include "physics/constants.hpp"

PhysicsObj::PhysicsObj(Model m, const linalg::aliases::float3& initPos,
                       const linalg::aliases::float3& initVel)
  : model{std::move(m)},
    // integrate back in time to discover the initial previous position
    pos{initPos},
    prevPos{initPos - initVel * Physics::deltaT +
            0.5 * constantForces / mass * Physics::deltaT * Physics::deltaT}
{
}

const linalg::aliases::float3& PhysicsObj::getPosition() const
{
    return pos;
}

const linalg::aliases::float3& PhysicsObj::getConstForce() const
{
    return constantForces;
}

void PhysicsObj::setConstForce(const linalg::aliases::float3& force)
{
    constantForces = force;
}

const Quaternion& PhysicsObj::getRotation() const
{
    return rotation;
}
void PhysicsObj::update()
{
    using namespace Physics;

    const linalg::aliases::float3 newPrevPos = pos;
    const linalg::aliases::float3 accelIntegral =
      deltaT * deltaT * (gravitationalAcceleration + constantForces / mass);
    pos = pos + pos - prevPos + accelIntegral;
    prevPos = newPrevPos;
}
