#pragma once

#include "util/queueChannel.hpp"
#include <vector>

class RigidBody;

class PhysicsEngine
{
  public:
    PhysicsEngine();
    void updateRigidBodies(std::vector<RigidBody>& rigidBodies);
};
