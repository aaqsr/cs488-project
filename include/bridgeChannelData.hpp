#pragma once

#include "physics/rigidBody.hpp"
#include "sim/waterSimulation.hpp"

#include <vector>

// This struct outlines data that the physics & simulation thread writes to
// and the render thread reads from. Hence the name: it is the main bridge
// between these two worlds.
struct BridgeChannelData
{
    HeightGrid<WaterSimulation::numRows, WaterSimulation::numCols> waterHeights;

    std::vector<RigidBodyData> physicsObjects;
};
