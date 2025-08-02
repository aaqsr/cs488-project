#pragma once

#include "physics/constants.hpp"
#include "sim/staggeredGrid.hpp"
#include "sim/waterHeightGrid.hpp"

#include <linalg.h>

struct SubTriangle;
class RigidBodyData;

class WaterSimulation
{
  public:
    using Real = float;
    using Real2 = linalg::aliases::float2;
    using Real3 = linalg::aliases::float3;

    constexpr static Real cellSize = Physics::WaterSim::cellSize;

    // TODO: come up with actual numbers
    constexpr static size_t numRows = 100;
    constexpr static size_t numCols = 80;

    constexpr static Real2 bottomLeftCornerWorldPos_xz{0.025F, 0.025F};
    constexpr static Real2 topRightCornerWorldPos_xz{
      (static_cast<Real>(numCols - 1) * cellSize) +
        bottomLeftCornerWorldPos_xz.x,
      (static_cast<Real>(numRows - 1) * cellSize) +
        bottomLeftCornerWorldPos_xz.y};

    // Grid spacing in meters.
    // Equal to cell size if we assume 1 meter = 1.0F in world space
    constexpr static Real deltaX = cellSize;

    // Some tiny loss of energy for velocity components
    // TODO: Come up with an actual value for this guy
    constexpr static Real velocityComponentDissipationConstant = 0.99985F;

    // TODO: Check for volume conservation every N timesteps?

    constexpr static Real3 upDirection_yHat = {0.0F, 1.0F, 0.0F};

    constexpr static Real decayRate_SolidsToFluids = 1.5F;
    static_assert(decayRate_SolidsToFluids > 0.0F);

    constexpr static Real Cdisplacement_SolidsToFluids = 1.0F;
    constexpr static Real Cadapt_SolidsToFluids = 0.2F;

    inline static Real adaptiveDeltaT = Physics::WaterSim::deltaT;

  private:
    StaggeredVelocityGrid<numRows, numCols> velocityGrid{};

    //
    //
    // required sub-steps based on CFL condition
    int currentSubStep = 0;
    int totalSubSteps = 1;
    Real accumulatedTime = 0.0;
    bool subStepInProgress = false;
    // calc. optimal sub-step size based on CFL condition.
    // Bound suggested in Fluid Simulation for Computer Graphics by R. Bridson
    // in section 12.3.
    // Incorrect computation. See WaterSimulation::calculateOptimalSubsteps
    // instead constexpr static float deltaTBoundAbove =
    //   cellSize / CS488Math::sqrt(gravitationalAccelerationMagnitude *
    //   maxDepth);

    // to ensure fixed deltaT is bounded above by the...bound given above, we
    // use a fractional value of the bound (0.2 is suggested by Fluid
    // Simulation for Computer Graphics by R. Bridson in section 12.3).
    // constexpr static float deltaT = 0.0002F;
    // constexpr static float deltaT = deltaTBoundAbove * 0.005F;
    // Other bounds on deltaT:
    // - t_n + deltaT < t_frame  [SCG by R.B. section 2.3]
    // -
    int calculateOptimalSubSteps(
      const HeightGrid<numRows, numCols>& heightGrid) const;
    // perform single sub-step update
    void
    performSingleSubStep(HeightGrid<numRows, numCols>& newHeightGrid,
                         const HeightGrid<numRows, numCols>& prevHeightGrid,
                         Real subDeltaT);
    //
    //

    // Add this to h_{i,j} each tick
    [[nodiscard]] Real calcHeightChangeIntegral(
      size_t i, size_t j, const HeightGrid<numRows, numCols>& heightGrid) const;

    void updateHeightsSemiImplicit(
      HeightGrid<numRows, numCols>& newHeightGrid,
      const HeightGrid<numRows, numCols>& prevHeightGrid);

    // TODO: could be static!
    // Returns (u_{u+1/2, j}, w_{i,j+1/2})
    [[nodiscard]] Real2 calcVelocityChangeIntegration(
      size_t i, size_t j, const HeightGrid<numRows, numCols>& heightGrid,
      const Real3& accelExt = {0.0F, 0.0F, 0.0F}) const;

    // TODO: make all these functions static and operate on an inputed grid
    // [[nodiscard]]
    // std::unique_ptr<StaggeredGrid<numRows, numCols>>
    void advectVelocities();

    void updateFluidWithTriangle(
      Real areaOfTriangle, const Real3& positionOfCentroid,
      const Real3& velocityOfCentroid,
      const Real3& relativeVelocityOfCentroidWRTFluid,
      const Real3& normalOfCentroid,
      HeightGrid<WaterSimulation::numRows, WaterSimulation::numCols>& heights);

    [[nodiscard]] Real3 computeFluidForceOnTriangle(
      const SubTriangle& subTriangle, const Real3& triangleVelocity,
      const HeightGrid<numRows, numCols>& heights) const;

    [[nodiscard]] Real3 getFluidVelocityAtPosition(const Real3& worldPos) const;

    // volume conservation + other stability hacks yay
    mutable int stabilityCheckCounter = 0;
    mutable Real previousTotalEnergy = 0.0F;
    void performStabilityCheck(const HeightGrid<numRows, numCols>& heightGrid);

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

    [[nodiscard]] static bool
    isPositionInWater(const Real3& pos,
                      const HeightGrid<WaterSimulation::numRows,
                                       WaterSimulation::numCols>& heights);

    void coupleWithRigidBodies(
      std::vector<RigidBodyData>& rigidBodies,
      HeightGrid<WaterSimulation::numRows, WaterSimulation::numCols>& heights);
};
