#include "physics/physicsEngine.hpp"
#include "linalg.h"
#include "physics/constants.hpp"
#include "physics/physicsObj.hpp"

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

void PhysicsEngine::update(std::vector<PhysicsObj>& objects)
{
    for (auto& o : objects) {
        verletIntegUpdate(o.prevPos, o.pos, o.getConstAccel());
    }
}
