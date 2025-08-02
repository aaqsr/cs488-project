#include "physics/physicsEngine.hpp"
#include "physics/constants.hpp"
#include "physics/rigidBody.hpp"
#include "util/error.hpp"
#include "util/logger.hpp"
#include "util/quaternion.hpp"

#include <linalg.h>

#include <iterator>
#include <sstream>
#include <string>

using Physics::RigidBody::deltaT;

namespace
{

void debugInertiaTensorPrint(
  const std::shared_ptr<const RigidBodyCharacteristics>& characteristics)
{
    static bool loggedBottleInertia = false;
    if (!loggedBottleInertia) {
        const linalg::aliases::float3x3& bodyInvInertia =
          characteristics->getInitInvInertia();
        auto bodyInertia =
          characteristics->getInertiaTensor().initialInertiaMatrix();

        std::stringstream ss;
        ss << "BODY CHARACTERISTICS DEBUG\n";
        ss << "Mass: " << characteristics->getInertiaTensor().getMass() << "\n";
        auto com = characteristics->getInertiaTensor().getCentreOfMass();
        ss << "Centre of mass: " << com.x << ", " << com.y << ", " << com.z
           << "\n";

        ss << "Scale: [" << characteristics->getScale().x << ", "
           << characteristics->getScale().y << ", "
           << characteristics->getScale().z << "]\n";

        ss << "Body inertia matrix:\n";
        ss << bodyInertia[0][0] << " " << bodyInertia[0][1] << " "
           << bodyInertia[0][2] << "\n";
        ss << bodyInertia[1][0] << " " << bodyInertia[1][1] << " "
           << bodyInertia[1][2] << "\n";
        ss << bodyInertia[2][0] << " " << bodyInertia[2][1] << " "
           << bodyInertia[2][2] << "\n";

        ss << "Body inverse inertia matrix:\n";
        ss << bodyInvInertia[0][0] << " " << bodyInvInertia[0][1] << " "
           << bodyInvInertia[0][2] << "\n";
        ss << bodyInvInertia[1][0] << " " << bodyInvInertia[1][1] << " "
           << bodyInvInertia[1][2] << "\n";
        ss << bodyInvInertia[2][0] << " " << bodyInvInertia[2][1] << " "
           << bodyInvInertia[2][2] << "\n";

        // Check for problematic values
        float minDiag =
          std::min({bodyInertia[0][0], bodyInertia[1][1], bodyInertia[2][2]});
        float maxDiag =
          std::max({bodyInertia[0][0], bodyInertia[1][1], bodyInertia[2][2]});
        float ratio = maxDiag / minDiag;

        ss << "Inertia diagonal range: [" << minDiag << ", " << maxDiag
           << "], ratio: " << ratio << "\n";

        if (ratio > 1000.0F) {
            ss << "WARNING: high inertia ratio issue\n";
        }

        if (minDiag < 1e-10F) {
            ss << "WARNING: small inertia values degenerate object\n";
        }

        Logger::GetInstance().log(ss.str());
        loggedBottleInertia = true;
    }
}

linalg::aliases::float3
findValidPosition(const AABB& aabb, const RigidBodyData& rigidBody,
                  const linalg::aliases::float3& desiredPosition)
{
    linalg::aliases::float3 offset =
      desiredPosition - rigidBody.getWorldPosition();

    const std::vector<Triangle>& triangles = rigidBody.getTriangles();

    // SHOULD NEVER HAPPEN
    if (triangles.empty()) {
        return linalg::aliases::float3{
          std::max(aabb.min.x, std::min(aabb.max.x, desiredPosition.x)),
          std::max(aabb.min.y, std::min(aabb.max.y, desiredPosition.y)),
          std::max(aabb.min.z, std::min(aabb.max.z, desiredPosition.z))};
    }

    // find the AABB at desired position
    linalg::aliases::float3 firstVertex = triangles[0].vertices[0] + offset;
    AABB desiredAABB{firstVertex, firstVertex};

    for (const Triangle& triangle : triangles) {
        for (int i = 0; i < 3; ++i) {
            linalg::aliases::float3 vertex = triangle.vertices[i] + offset;
            desiredAABB.expand(vertex);
        }
    }

    // correction needed to keep AABB within bounds
    linalg::aliases::float3 correction{0.0F, 0.0F, 0.0F};

    if (desiredAABB.min.x < aabb.min.x) {
        correction.x = aabb.min.x - desiredAABB.min.x;
    } else if (desiredAABB.max.x > aabb.max.x) {
        correction.x = aabb.max.x - desiredAABB.max.x;
    }

    if (desiredAABB.min.y < aabb.min.y) {
        correction.y = aabb.min.y - desiredAABB.min.y;
    } else if (desiredAABB.max.y > aabb.max.y) {
        correction.y = aabb.max.y - desiredAABB.max.y;
    }

    if (desiredAABB.min.z < aabb.min.z) {
        correction.z = aabb.min.z - desiredAABB.min.z;
    } else if (desiredAABB.max.z > aabb.max.z) {
        correction.z = aabb.max.z - desiredAABB.max.z;
    }

    return desiredPosition + correction;
}

} // namespace

// TODO: Damping??
void PhysicsEngine::simulateRigidBody(RigidBodyData& out,
                                      const RigidBodyData& prev)
{
    //
    // Linear dynamics
    //

    // Verlet update
    float mass = prev.characteristics->getInertiaTensor().getMass();

    // accumulated force Verlet
    out.worldPosition = prev.worldPosition +
                        (prev.worldPosition - prev.prevWorldPosition) +
                        (deltaT * deltaT * prev.accumulatedForce / mass);
    out.accumulatedForce = {0.0F, 0.0F, 0.0F};

    // gravity
    out.worldPosition.y -=
      Physics::gravitationalAccelerationMagnitude * deltaT * deltaT;

    out.prevWorldPosition = prev.worldPosition;

    //
    // Rotational Dynamics
    //
    auto newAngularMomentum =
      prev.angularMomentum + prev.accumulatedTorque * deltaT;
    out.angularMomentum = newAngularMomentum;
    out.accumulatedTorque = {0.0F, 0.0F, 0.0F};

    // Derivative trick method (might be cheaper since no trig functions).
    // Sources:
    // https://www.euclideanspace.com/physics/kinematics/angularvelocity/
    // https://www.researchgate.net/publication/46422891_Robust_rotational-velocity-Verlet_integration_methods
    auto R = prev.orientation.toMatrix3x3();
    auto bodyInvInertia = prev.characteristics->getInitInvInertia();
    auto invInertiaWorldFrame =
      mul(mul(R, bodyInvInertia), linalg::transpose(R));

    auto angularVelocityWorldFrame =
      mul(invInertiaWorldFrame, newAngularMomentum);

    Quaternion deltaAngularOrientationWorldFrame = Quaternion{
      linalg::aliases::float4{angularVelocityWorldFrame.x,
                              angularVelocityWorldFrame.y,
                              angularVelocityWorldFrame.z, 0.0F}
    };

    Quaternion qDerivativeWrtTime =
      (prev.orientation * deltaAngularOrientationWorldFrame) * 0.5F;

    // forward Euler integ. (could use midpoint if needed??)
    out.orientation = prev.orientation + qDerivativeWrtTime * deltaT;

    // unit quaternions
    out.orientation.normalize();

    out.invalidateTriangleCache();
}

PhysicsEngine::PhysicsEngine(
  Receiver<std::vector<PhysicsEngineReceiverData>>& recv)
  : channel{recv}
{
}

void PhysicsEngine::updateRigidBodies(
  std::vector<RigidBodyData>& rigidBodies,
  const std::vector<RigidBodyData>& prevRigidBodies)
{
    if (prevRigidBodies.size() > rigidBodies.size()) {
        for (size_t newElemIdx = rigidBodies.size();
             newElemIdx < prevRigidBodies.size(); ++newElemIdx)
        {
            rigidBodies.push_back(prevRigidBodies[newElemIdx]);
        }
    }

    if (channel.isMessageReady()) {
        auto msg = channel.receive();
        for (auto& cmd : msg.getBuffer_MutableDangerous()) {
            auto characteristics = std::make_shared<RigidBodyCharacteristics>(
              cmd.model, cmd.scale, cmd.density);

            rigidBodyCharacteristics.push_back(characteristics);

            rigidBodies.emplace_back(characteristics, cmd.initPos, cmd.initVel,
                                     cmd.initAngVel);
            rigidBodies.back().applyForce(cmd.initForce, cmd.initForceContact);

            debugInertiaTensorPrint(rigidBodies.back().characteristics);
        }
    }

    for (size_t i = 0; i < prevRigidBodies.size(); ++i) {
        const auto& prevBody = prevRigidBodies[i];
        auto& body = rigidBodies[i];

        if (body.characteristics.get() != prevBody.characteristics.get()) {
            throw IrrecoverableError{"Characteristic ptr mismatch"};
        }

        if (!body.enabled) {
            continue;
        }
        if (body.worldPosition.y <= -0.5F) {
            body.enabled = false;
            continue;
        }

        simulateRigidBody(body, prevBody);

        if (thePoolLimits.contains(body.worldPosition) &&
            body.worldPosition.y <= poolHeight)
        {
            keepWithinAABB(thePoolLimits, body);
        } else {
            keepWithinAABB(theWorldLimits, body);
        }

        if (std::isnan(body.worldPosition.x + body.worldPosition.y +
                       body.worldPosition.z))
        {
            throw IrrecoverableError{"Rigid Body position is NAN yay"};
        }
    }
}

// TODO: please oh god please refactor this please
void PhysicsEngine::keepWithinAABB(AABB aabb, RigidBodyData& rigidBody)
{
    linalg::aliases::float3 validPosition =
      findValidPosition(aabb, rigidBody, rigidBody.getWorldPosition());

    linalg::aliases::float3 correction =
      validPosition - rigidBody.getWorldPosition();
    float correctionMagnitude = linalg::length(correction);

    if (correctionMagnitude < 1e-6F) {
        return; // already in valid position
    }

    // position correction
    rigidBody.worldPosition = validPosition;

    // rotation constraint: if we're at a boundary, limit angular momentum to
    // prevent the object from rotating outside the bounds
    AABB currentAABB = rigidBody.computeAABB();

    // are we touching any boundary
    const float tolerance = 1e-4F;
    bool touchingBoundary =
      (std::abs(currentAABB.min.x - aabb.min.x) < tolerance ||
       std::abs(currentAABB.max.x - aabb.max.x) < tolerance) ||
      (std::abs(currentAABB.min.y - aabb.min.y) < tolerance ||
       std::abs(currentAABB.max.y - aabb.max.y) < tolerance) ||
      (std::abs(currentAABB.min.z - aabb.min.z) < tolerance ||
       std::abs(currentAABB.max.z - aabb.max.z) < tolerance);

    // TODO: Doesn't work :(
    if (touchingBoundary) {
        // we project angular velocity to only allow rotations that keep the
        // object in bounds
        auto R = rigidBody.orientation.toMatrix3x3();
        auto bodyInvInertia = rigidBody.characteristics->getInitInvInertia();
        auto invInertiaWorldFrame =
          mul(mul(R, bodyInvInertia), linalg::transpose(R));
        auto currentAngularVel =
          mul(invInertiaWorldFrame, rigidBody.angularMomentum);

        // we test small rotations in each axis to see which ones cause
        // violations
        const float testAngle = 1e-5F;
        linalg::aliases::float3 projectedAngularVel = currentAngularVel;

        if (std::abs(currentAngularVel.x) > 1e-6F) {
            Quaternion testRotX = Quaternion::fromAxisAngle(
              linalg::aliases::float3{1.0F, 0.0F, 0.0F},
              testAngle * (currentAngularVel.x > 0 ? 1.0F : -1.0F));
            Quaternion testOrientation = rigidBody.orientation * testRotX;
            testOrientation.normalize();

            Quaternion originalOrientation = rigidBody.orientation;
            rigidBody.orientation = testOrientation;
            rigidBody.invalidateTriangleCache();

            linalg::aliases::float3 testValidPos =
              findValidPosition(aabb, rigidBody, rigidBody.getWorldPosition());
            bool wouldViolate =
              linalg::length(testValidPos - rigidBody.getWorldPosition()) >
              1e-6F;

            rigidBody.orientation = originalOrientation;
            rigidBody.invalidateTriangleCache();

            if (wouldViolate) {
                projectedAngularVel.x = 0.0F;
            }
        }
        if (std::abs(currentAngularVel.y) > 1e-6F) {
            Quaternion testRotY = Quaternion::fromAxisAngle(
              linalg::aliases::float3{0.0F, 1.0F, 0.0F},
              testAngle * (currentAngularVel.y > 0 ? 1.0F : -1.0F));
            Quaternion testOrientation = rigidBody.orientation * testRotY;
            testOrientation.normalize();

            Quaternion originalOrientation = rigidBody.orientation;
            rigidBody.orientation = testOrientation;
            rigidBody.invalidateTriangleCache();

            linalg::aliases::float3 testValidPos =
              findValidPosition(aabb, rigidBody, rigidBody.getWorldPosition());
            bool wouldViolate =
              linalg::length(testValidPos - rigidBody.getWorldPosition()) >
              1e-6F;

            rigidBody.orientation = originalOrientation;
            rigidBody.invalidateTriangleCache();

            if (wouldViolate) {
                projectedAngularVel.y = 0.0F;
            }
        }
        if (std::abs(currentAngularVel.z) > 1e-6F) {
            Quaternion testRotZ = Quaternion::fromAxisAngle(
              linalg::aliases::float3{0.0F, 0.0F, 1.0F},
              testAngle * (currentAngularVel.z > 0 ? 1.0F : -1.0F));
            Quaternion testOrientation = rigidBody.orientation * testRotZ;
            testOrientation.normalize();

            Quaternion originalOrientation = rigidBody.orientation;
            rigidBody.orientation = testOrientation;
            rigidBody.invalidateTriangleCache();

            linalg::aliases::float3 testValidPos =
              findValidPosition(aabb, rigidBody, rigidBody.getWorldPosition());
            bool wouldViolate =
              linalg::length(testValidPos - rigidBody.getWorldPosition()) >
              1e-6F;

            rigidBody.orientation = originalOrientation;
            rigidBody.invalidateTriangleCache();

            if (wouldViolate) {
                projectedAngularVel.z = 0.0F;
            }
        }

        // and now we can convert the corrected projected angular velocity back
        // to angular momentum
        auto inertiaWorldFrame = linalg::inverse(invInertiaWorldFrame);
        rigidBody.angularMomentum = mul(inertiaWorldFrame, projectedAngularVel);
    }

    rigidBody.invalidateTriangleCache();
}
