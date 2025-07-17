#pragma once

#include "linalg.h"
#include "sim/staggeredGrid.hpp"
#include "sim/waterHeightGrid.hpp"

class WaterSimulation
{
  public:
    // TODO: come up with actual numbers
    constexpr static size_t numRows = 100;
    constexpr static size_t numCols = 80;

    constexpr static float cellSize = 0.05F;

    // Grid spacing in meters.
    // Equal to cell size if we assume 1 meter = 1.0F in world space
    constexpr static float deltaX = cellSize;

    // TODO: the paper suggests a better maxDepth in section 2.1.5
    // TODO: should we/can we lower the base depth of 1.0 further?
    constexpr static float maxDepth = 1.5F;

    // Some tiny loss of energy for velocity components
    // TODO: Come up with an actual value for this guy
    constexpr static float velocityComponentDissipationConstant = 0.99985F;

    // TODO: Check for volume conservation every N timesteps?

  private:
    StaggeredVelocityGrid<numRows, numCols> velocityGrid;

    // Add this to h_{i,j} each tick
    [[nodiscard]]
    float calcHeightChangeIntegral(
      size_t i, size_t j, const HeightGrid<numRows, numCols>& heightGrid) const;

    // TODO: could be static!
    // Returns (u_{u+1/2, j}, w_{i,j+1/2})
    [[nodiscard]]
    linalg::aliases::float2 calcVelocityChangeIntegration(
      size_t i, size_t j, const HeightGrid<numRows, numCols>& heightGrid,
      const linalg::aliases::float3& accelExt = {0.0F, 0.0F, 0.0F}) const;

    // TODO: make all these functions static and operate on an inputed grid
    // [[nodiscard]]
    // std::unique_ptr<StaggeredGrid<numRows, numCols>>
    void advectVelocities();

  public:
    WaterSimulation();

    WaterSimulation(const WaterSimulation&) = delete;
    WaterSimulation(WaterSimulation&&) = delete;
    WaterSimulation& operator=(const WaterSimulation&) = delete;
    WaterSimulation& operator=(WaterSimulation&&) = delete;

    static void
    setInitConditions(HeightGrid<WaterSimulation::numRows,
                                 WaterSimulation::numCols>& heightGrid);

    void update(HeightGrid<numRows, numCols>& newHeightGrid,
                const HeightGrid<numRows, numCols>& prevHeightGrid);
};
