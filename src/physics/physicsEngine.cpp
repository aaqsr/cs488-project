#include "physics/physicsEngine.hpp"
#include "physics/collision.hpp"
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

        if (SuperCollider::thePoolLimits.contains(body.worldPosition) &&
            body.worldPosition.y <= SuperCollider::poolHeight)
        {
            SuperCollider::keepWithinAABB(SuperCollider::thePoolLimits, body);
        } else {
            SuperCollider::keepWithinAABB(SuperCollider::theWorldLimits, body);
        }

        if (std::isnan(body.worldPosition.x + body.worldPosition.y +
                       body.worldPosition.z))
        {
            throw IrrecoverableError{"Rigid Body position is NAN yay"};
        }
    }
    SuperCollider::processAABBCollisions(rigidBodies);
}

