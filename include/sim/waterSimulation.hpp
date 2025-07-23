#pragma once

#include "sim/staggeredGrid.hpp"
#include "sim/waterHeightGrid.hpp"

#include <linalg.h>

class RigidBody;

class WaterSimulation
{
  public:
    // TODO: come up with actual numbers
    constexpr static size_t numRows = 100;
    constexpr static size_t numCols = 80;

    constexpr static float cellSize = 0.05F;
    constexpr static linalg::aliases::float2 bottomLeftCornerWorldPos_xz{
      0.025F, 0.025F};
    constexpr static linalg::aliases::float2 topRightCornerWorldPos_xz{
      (static_cast<float>(numCols - 1) * cellSize) +
        bottomLeftCornerWorldPos_xz.x,
      (static_cast<float>(numRows - 1) * cellSize) +
        bottomLeftCornerWorldPos_xz.y};

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

    constexpr static linalg::aliases::float3 upDirection_yHat = {0.0F, 1.0F,
                                                                 0.0F};

    constexpr static float decayRate_SolidsToFluids = 1.0F;
    static_assert(decayRate_SolidsToFluids > 0.0F);

    constexpr static float Cdisplacement_SolidsToFluids = 1.0F;
    constexpr static float Cadapt_SolidsToFluids = 0.2F;

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

    [[nodiscard]] static bool doesObjectCollideWithWater(
      const linalg::aliases::float3& pos,
      const HeightGrid<WaterSimulation::numRows, WaterSimulation::numCols>&
        heights);

    void updateFluidWithRigidBody(
      float areaOfTriangle, const linalg::aliases::float3& positionOfCentroid,
      const linalg::aliases::float3& velocityOfCentroid,
      const linalg::aliases::float3& relativeVelocityOfCentroidWRTFluid,
      const linalg::aliases::float3& normalOfCentroid,
      HeightGrid<WaterSimulation::numRows, WaterSimulation::numCols>& heights);
};
