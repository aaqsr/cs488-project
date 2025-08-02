#pragma once

#include <linalg.h>

#include <memory>
#include <vector>

class Model;
class RigidBodyCharacteristics;
class RigidBodyData;
template <typename T>
class Receiver;

// Can easily be converted into a command like object if we also need to be able
// to remove objects by command or something
struct PhysicsEngineReceiverData
{
    std::shared_ptr<Model> model;
    linalg::aliases::float3 scale{1.0F};
    linalg::aliases::float3 initPos{0.0F};
    linalg::aliases::float3 initVel{0.0F};
    linalg::aliases::float3 initAngVel{0.0F};

    linalg::aliases::float3 initForce{0.0F};
    linalg::aliases::float3 initForceContact{0.0F};

    float density = 1.0F;
};

class PhysicsEngine
{
    constexpr static float linearDamping = 1.0F;
    constexpr static float angularDamping = 1.0F;

    Receiver<std::vector<PhysicsEngineReceiverData>>& channel;

    std::vector<std::shared_ptr<RigidBodyCharacteristics>>
      rigidBodyCharacteristics;

    void simulateRigidBody(RigidBodyData& out, const RigidBodyData& prev);

  public:
    PhysicsEngine(Receiver<std::vector<PhysicsEngineReceiverData>>& recv);
    void updateRigidBodies(std::vector<RigidBodyData>& rigidBodies,
                           const std::vector<RigidBodyData>& prevRigidBodies);
};
