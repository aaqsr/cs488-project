#pragma once

#include <vector>

class PhysicsObj;

class PhysicsEngine
{
  public:
    PhysicsEngine();
    void update(std::vector<PhysicsObj>& physicsObjects);
};
