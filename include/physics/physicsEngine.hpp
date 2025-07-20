#pragma once

#include <vector>

class RigidBody;

class PhysicsEngine
{
  public:
    PhysicsEngine();
    void updateRigidBodies(std::vector<RigidBody>& rigidBodies);
};
