#include "physics/physicsEngine.hpp"
#include "linalg.h"
#include "physics/constants.hpp"
#include "physics/rigidBody.hpp"

namespace
{

void verletIntegUpdate(linalg::aliases::float3& prevPos,
                       linalg::aliases::float3& pos,
                       const linalg::aliases::float3& accel)
{
    using namespace Physics;

    const linalg::aliases::float3 newPrevPos = pos;
    const linalg::aliases::float3 accelIntegral = deltaT * deltaT * accel;
    const linalg::aliases::float3 newPos = pos + pos - prevPos + accelIntegral;

    prevPos = newPrevPos;
    pos = newPos;
}

} // namespace

PhysicsEngine::PhysicsEngine() = default;

void PhysicsEngine::updateRigidBodies(std::vector<RigidBody>& objects)
{
    for (auto& o : objects) {
        verletIntegUpdate(o.prevGlobalCentreOfMassPos, o.globalCentreOfMassPos,
                          o.getTotalAcceleration());

        // physics stuff here

        o.clearAccumulatedForceAndTorque();
    }
}
