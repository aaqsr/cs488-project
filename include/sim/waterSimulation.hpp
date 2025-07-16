#pragma once

#include "linalg.h"
#include "sim/staggeredGrid.hpp"
#include "sim/waterHeightGrid.hpp"
#include "util/channel.hpp"
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
    constexpr static float deltaT = deltaTBoundAbove * 0.1F;
    // Other bounds on deltaT:
    // - t_n + deltaT < t_frame  [SCG by R.B. section 2.3]
    // -

    // TODO: HOW TO BOUND THE WAVE SIZE AAAAA
    // TODO: come up with an actual min speed value if we even want this...it
    // would break conservation of energy?
    constexpr static float minSpeedComponent = 0.0F;

    constexpr static float maxSpeedComponent = ([]() {
        constexpr float alpha = 0.5F;
        return alpha * cellSize / deltaT;
    })();

    // Some tiny loss of energy for velocity components
    // TODO: Come up with an actual value for this guy
    constexpr static float velocityComponentDissipationConstant = 0.99985F;

    // TODO: Check for volume conservation every N timesteps?

  private:
    StaggeredVelocityGrid<numRows, numCols> velocityGrid{minSpeedComponent,
                                                         maxSpeedComponent};

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

    std::atomic<bool> isPlaying{false};

    Sender<HeightGrid<numRows, numCols>>* channel = nullptr;

  public:
    WaterSimulation();

    WaterSimulation(const WaterSimulation&) = delete;
    WaterSimulation(WaterSimulation&&) = delete;
    WaterSimulation& operator=(const WaterSimulation&) = delete;
    WaterSimulation& operator=(WaterSimulation&&) = delete;

    void update();
    void togglePlay();
    void attachSenderChannel(Sender<HeightGrid<numRows, numCols>>* s);
};
