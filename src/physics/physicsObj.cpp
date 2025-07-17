#include "physics/physicsObj.hpp"
#include "physics/constants.hpp"

PhysicsObj::PhysicsObj(const linalg::aliases::float3& initPos,
                       const linalg::aliases::float3& initVel)
  : // integrate back in time to discover the initial previous position
    pos{initPos},
    prevPos{initPos - initVel * Physics::deltaT +
            0.5 * constantAcceleration * Physics::deltaT * Physics::deltaT}
{
}

const linalg::aliases::float3& PhysicsObj::getPosition() const
{
    return pos;
}

const linalg::aliases::float3& PhysicsObj::getConstAccel() const
{
    return constantAcceleration;
}

void PhysicsObj::setConstForce(const linalg::aliases::float3& force)
{
    constantAcceleration = force / mass;
}

const linalg::aliases::float3& PhysicsObj::getPrevPosition() const
{
    return prevPos;
}

