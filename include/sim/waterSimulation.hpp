#pragma once

#include "linalg.h"
#include "sim/staggeredGrid.hpp"
#include "sim/waterMesh.hpp"
#include "util/math.hpp"

class WaterSimulation
{
  public:
    // TODO: come up with actual numbers
    constexpr static size_t numRows = 100;
    constexpr static size_t numCols = 80;

    constexpr static float cellSize = 0.05F;

    constexpr static float gravitationalAcceleration = 9.80665F;

    // Grid spacing in meters.
    // Equal to cell size if we assume 1 meter = 1.0F in world space
    constexpr static float deltaX = cellSize;

    // TODO: the paper suggests a better maxDepth in section 2.1.5
    // TODO: should we/can we lower the base depth of 1.0 further?
    constexpr static float maxDepth = 1.5F;

    // bound suggested in Fluid Simulation for Computer Graphics by R. Bridson
    // in section 12.3.
    constexpr static float deltaTBoundAbove =
      cellSize / CS488Math::sqrt(gravitationalAcceleration * maxDepth);

    // to ensure fixed deltaT is bounded above by the...bound given above, we
    // use a fractional value of the bound (0.2 is suggested by Fluid
    // Simulation for Computer Graphics by R. Bridson in section 12.3).
    // We use the current value discovered via trial and error.
    constexpr static float deltaT = deltaTBoundAbove * 0.2F;
    // Other bounds on deltaT:
    // - t_n + deltaT < t_frame  [SCG by R.B. section 2.3]
    // -

    constexpr static float maxSpeedClamp = ([]() {
        constexpr float alpha = 0.5F;
        return alpha * cellSize / deltaT;
    })();

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
    void advectVelocities(float deltaTime);

    bool isPlaying = false;

  public:
    WaterSimulation();

    void update();
    void draw(Shader::BindObject& shader,
              const linalg::aliases::float3& cameraPos);
    void togglePlay();
};
