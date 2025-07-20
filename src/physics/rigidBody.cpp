#include "physics/rigidBody.hpp"
#include "linalg.h"
#include "physics/constants.hpp"

RigidBody::RigidBody(const linalg::aliases::float3& initPos,
                     const linalg::aliases::float3& initVel)
  : // integrate back in time to discover the initial previous position
    globalCentreOfMassPos{initPos},
    prevGlobalCentreOfMassPos{initPos - initVel * Physics::deltaT +
                              0.5 * constantAcceleration * Physics::deltaT *
                                Physics::deltaT}
{
}

const linalg::aliases::float3& RigidBody::getCentreOfMass() const
{
    return globalCentreOfMassPos;
}

void RigidBody::setConstForce(const linalg::aliases::float3& force)
{
    constantAcceleration = force / mass;
}

void RigidBody::clearAccumulatedForceAndTorque()
{
    accumulatedForce = linalg::aliases::float3{0.0F, 0.0F, 0.0F};
}

linalg::aliases::float3 RigidBody::getTotalAcceleration() const
{
    return constantAcceleration + accumulatedForce / mass;
}

