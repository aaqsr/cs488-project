#pragma once

#include "linalg.h"
#include "sim/staggeredGrid.hpp"
#include "sim/waterMesh.hpp"
#include "util/math.hpp"

class WaterSimulation
{
  public:
    // TODO: come up with actual numbers
    constexpr static size_t numRows = 50;
    constexpr static size_t numCols = 30;

    constexpr static float cellSize = 0.2F;

    constexpr static float gravitationalAcceleration = 9.80665F;

    // Grid spacing in meters.
    // Equal to cell size if we assume 1 meter = 1.0F in world space
    constexpr static float deltaX = cellSize;

    // TODO: actually use this to bound the depth maybe...
    // TODO: the paper suggests a better maxDepth in section 2.1.5
    constexpr static float maxDepth = 1.0F;

    // bound suggested in Fluid Simulation for Computer Graphics by R. Bridson
    // in section 12.3.
    constexpr static float deltaTBoundAbove =
      cellSize / CS488Math::sqrt(gravitationalAcceleration * maxDepth);

    // to ensure fixed deltaT is bounded above by the...bound given above, we
    // use a fractional value of the bound (again as suggested by Fluid
    // Simulation for Computer Graphics by R. Bridson in section 12.3)
    constexpr static float deltaT = deltaTBoundAbove * 0.07F;

  private:
    StaggeredGrid<numRows, numCols> grid;
    WaterMesh<numRows, numCols, cellSize> mesh;

    // Add this to h_{i,j} each tick
    [[nodiscard]]
    float calcHeightChangeIntegral(size_t i, size_t j, float deltaTime) const;

    // Returns (u_{u+1/2, j}, w_{i,j+1/2})
    [[nodiscard]]
    linalg::aliases::float2 calcVelocityChangeIntegration(
      size_t i, size_t j, float deltaTime,
      const linalg::aliases::float3& accelExt = {0.0F, 0.0F, 0.0F}) const;

    // TODO: make deltaTime a fixed value. And pick that value...
    // TODO: make all these functions static and operate on an inputed grid
    // [[nodiscard]]
    // std::unique_ptr<StaggeredGrid<numRows, numCols>>
    void advectVelocities(const StaggeredGrid<numRows, numCols>& currentGrid,
                          float deltaTime);

    [[nodiscard]]
    static float getMaxSpeedClamp(float deltaTime);

    bool isPlaying = false;

  public:
    WaterSimulation();

    void update();
    void draw(Shader::BindObject& shader);
    void togglePlay();
};
