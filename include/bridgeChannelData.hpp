#pragma once

#include "sim/waterSimulation.hpp"

// This struct outlines data that the physics thread writes to and the render
// thread reads from.
// Hence the name: it is the main bridge between these two worlds.
struct BridgeChannelData
{
    HeightGrid<WaterSimulation::numRows, WaterSimulation::numCols> waterHeights;
};
