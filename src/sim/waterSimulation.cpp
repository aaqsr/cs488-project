#include "sim/waterSimulation.hpp"
#include "linalg.h"
#include "physics/constants.hpp"
#include "physics/rigidBody.hpp"
#include "physics/rigidBodyMesh.hpp"
#include "sim/waterHeightGrid.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

using Physics::WaterSim::deltaT;

namespace
{

template <size_t rows, size_t cols>
[[nodiscard]]
float interpolate(const std::array<float, rows * cols>& field,
                  const linalg::aliases::float2& pos)
{
    // Boundary Conditions:
    auto getFieldValue = [&field](int i, int j) -> float {
        // reflection
        i = std::max(i, 0);
        j = std::max(j, 0);

        if (i >= static_cast<int>(cols)) {
            i = cols - 1;
        }
        if (j >= static_cast<int>(rows)) {
            j = rows - 1;
        }

        return field[(j * cols) + i];
    };

    const int i0 = static_cast<int>(std::floor(pos.x));
    const int j0 = static_cast<int>(std::floor(pos.y));
    const int i1 = i0 + 1;
    const int j1 = j0 + 1;

    const float tx = pos.x - static_cast<float>(i0);
    const float ty = pos.y - static_cast<float>(j0);

    const float v00 = getFieldValue(i0, j0);
    const float v10 = getFieldValue(i1, j0);
    const float v01 = getFieldValue(i0, j1);
    const float v11 = getFieldValue(i1, j1);

    // linear interp in x
    const float lerpX1 = v00 + ((v10 - v00) * tx);
    const float lerpX2 = v01 + ((v11 - v01) * tx);

    // linear interp in y
    return lerpX1 + ((lerpX2 - lerpX1) * ty);
}

// float calculateAdjustedHeight(
//   size_t i, size_t j,
//   const HeightGrid<WaterSimulation::numRows, WaterSimulation::numCols>&
//     heightGrid,
//   float baseHeight)
// {
//     // Stability enhancement from section 2.1.5
//     // hadj = max(0, (h_i+1,j + h_i-1,j + h_i,j+1 + h_i,j-1)/4 - havgmax)
//     // havgmax = β * Δx / (g * Δt)
//
//     constexpr float beta = 2.0F;
//     constexpr float havgmax =
//       beta * WaterSimulation::deltaX /
//       (WaterSimulation::gravitationalAcceleration * WaterSimulation::deltaT);
//
//     // Calculate average height of neighbors
//     float neighborSum = 0.0F;
//     int neighborCount = 0;
//
//     // Right neighbor (i+1, j)
//     if (i + 1 < WaterSimulation::numRows) {
//         neighborSum += heightGrid.getWaterHeight(i + 1, j);
//         neighborCount++;
//     }
//
//     // Left neighbor (i-1, j)
//     if (i > 0) {
//         neighborSum += heightGrid.getWaterHeight(i - 1, j);
//         neighborCount++;
//     }
//
//     // Bottom neighbor (i, j+1)
//     if (j + 1 < WaterSimulation::numCols) {
//         neighborSum += heightGrid.getWaterHeight(i, j + 1);
//         neighborCount++;
//     }
//
//     // Top neighbor (i, j-1)
//     if (j > 0) {
//         neighborSum += heightGrid.getWaterHeight(i, j - 1);
//         neighborCount++;
//     }
//
//     // For boundary cells, use available neighbors
//     float avgNeighborHeight =
//       (neighborCount > 0) ? neighborSum / neighborCount : baseHeight;
//
//     // Calculate hadj
//     float hadj = std::max(0.0F, avgNeighborHeight - havgmax);
//
//     // Return adjusted height: h_bar - hadj
//     return baseHeight - hadj;
// }

std::pair<int, int> getClosestGridPoint(float x, float z)
{
    float gridX = (x - WaterSimulation::bottomLeftCornerWorldPos_xz.x) /
                  WaterSimulation::cellSize;
    float gridZ = (z - WaterSimulation::bottomLeftCornerWorldPos_xz.y) /
                  WaterSimulation::cellSize;

    int i = std::clamp(static_cast<int>(std::round(gridZ)), 0,
                       static_cast<int>(WaterSimulation::numRows - 1));
    int j = std::clamp(static_cast<int>(std::round(gridX)), 0,
                       static_cast<int>(WaterSimulation::numCols - 1));

    return {i, j};
}

} // namespace

WaterSimulation::WaterSimulation()
  : velocityGrid{Physics::WaterSim::waterSimulationMaxSpeedComponent}
{
}

void WaterSimulation::setInitConditions(
  HeightGrid<WaterSimulation::numRows, WaterSimulation::numCols>& heightGrid)
{
    constexpr static auto fn = [](float x, float y) {
        // Gaussian hump function
        const float sigma = 0.25F;    // width of the hump
        const float amplitude = 0.5F; // height of the hump
        float r_squared = (x * x) + (y * y);
        return amplitude * std::exp(-r_squared / (2.0F * sigma * sigma));
    };

    for (size_t i = 0; i < WaterSimulation::numRows; ++i) {
        for (size_t j = 0; j < WaterSimulation::numCols; ++j) {
            if (((i >= WaterSimulation::numRows / 4) &&
                 (i < (3 * WaterSimulation::numRows) / 4)) &&
                ((j >= WaterSimulation::numCols / 4) &&
                 (j < (3 * WaterSimulation::numCols) / 4)))
            {
                float x = (static_cast<float>(i) -
                           (WaterSimulation::numRows * 0.5F) + 0.5F) *
                          WaterSimulation::cellSize;
                float y = (static_cast<float>(j) -
                           (WaterSimulation::numCols * 0.5F) + 0.5F) *
                          WaterSimulation::cellSize;

                // add the hump to the base level of 1.0
                heightGrid.setWaterHeight(i, j, 1.0F + fn(x, y),
                                          WaterSimulation::maxDepth);
            } else {
                heightGrid.setWaterHeight(i, j, 1.0F,
                                          WaterSimulation::maxDepth);
            }
        }
    }
}

void WaterSimulation::update(HeightGrid<numRows, numCols>& newHeightGrid,
                             const HeightGrid<numRows, numCols>& prevHeightGrid)
{
    advectVelocities();

    for (size_t i = 0; i < numRows; ++i) {
        for (size_t j = 0; j < numCols; ++j) {
            const float newHeight =
              prevHeightGrid.getWaterHeight(i, j) +
              calcHeightChangeIntegral(i, j, prevHeightGrid);
            newHeightGrid.setWaterHeight(i, j, newHeight, maxDepth);
        }
    }

    {
        // corner boundary condition to prevent corner (0, 0) from getting
        // stuck to init height throughout simulation
        // TODO: THIS IS A HACK. Impl properly?
        newHeightGrid.setWaterHeight(0, 0, newHeightGrid.getWaterHeight(0, 1),
                                     maxDepth);
    }
    // for (size_t i = 0; i < numRows; ++i) {
    //   grid.setWaterHeight(i, 0, 0.0F, maxDepth);
    // }
    // for (size_t j = 0; j < numCols; ++j) {
    //   grid.setWaterHeight(0, j, 0.0F, maxDepth);
    // }

    for (size_t i = 0; i < numRows; ++i) {
        for (size_t j = 0; j < numCols; ++j) {
            const auto& [deltaU, deltaW] =
              calcVelocityChangeIntegration(i, j, prevHeightGrid);

            const float newU =
              velocityGrid.getVelocity_u_i_plus_half_j(i, j) + deltaU;
            velocityGrid.setVelocity_u_i_plus_half_j(
              i, j, velocityComponentDissipationConstant * newU);

            const float newW =
              velocityGrid.getVelocity_w_i_j_plus_half(i, j) + deltaW;
            velocityGrid.setVelocity_w_i_j_plus_half(
              i, j, velocityComponentDissipationConstant * newW);
        }
    }

    // Boundary Conditions:
    // paper suggests setting boundary velocites to 0 at end of time step.
    // Why at the end and not before when we are already iterating through the
    // grid? idk but i am not about to tempt fate here...
    for (size_t j = 0; j < numCols; ++j) {
        // top wall
        velocityGrid.setVelocity_w_i_j_plus_half(0, j, 0.0F);
        // bottom wall
        velocityGrid.setVelocity_w_i_j_plus_half(numRows, j, 0.0F);
    }

    for (size_t i = 0; i < numRows; ++i) {
        // left wall
        velocityGrid.setVelocity_u_i_plus_half_j(i, 0, 0.0F);
        // right wall
        velocityGrid.setVelocity_u_i_plus_half_j(i, numCols, 0.0F);
    }
}

float WaterSimulation::calcHeightChangeIntegral(
  size_t i, size_t j, const HeightGrid<numRows, numCols>& heightGrid) const
{
    // Both evaluated in the upwind direction
    // TODO: why capturing one but not the other??
    const auto hBar_i_plus_half_j =
      [&heightGrid](const decltype(velocityGrid)& grid, size_t i,
                    size_t j) -> float {
        // Boundary conditions:
        // if at right edge, use current cell height
        if (i + 1 >= numRows) {
            return heightGrid.getWaterHeight(i, j);
        }

        return (grid.getVelocity_u_i_plus_half_j(i, j) <= 0)
                 ? heightGrid.getWaterHeight(i + 1, j)
                 : heightGrid.getWaterHeight(i, j);
    };
    const auto hBar_i_j_plus_half =
      [&heightGrid](const decltype(velocityGrid)& grid, size_t i,
                    size_t j) -> float {
        if (j + 1 >= numCols) {
            return heightGrid.getWaterHeight(i, j);
        }
        return (grid.getVelocity_w_i_j_plus_half(i, j) <= 0)
                 ? heightGrid.getWaterHeight(i, j + 1)
                 : heightGrid.getWaterHeight(i, j);
    };

    // TODO: Stability improvement in section 2.1.5 final paragraph
    // constexpr float beta = 2.0F;
    // const float h_avgmax = (beta * deltaX) / (gravitationalAcceleration *
    // deltaTime);

    // Boundary conditions: left boundary
    float uDirectionNumerator = 0.0F;
    if (i > 0) {
        uDirectionNumerator =
          (hBar_i_plus_half_j(velocityGrid, i, j) *
           velocityGrid.getVelocity_u_i_plus_half_j(i, j)) -
          (hBar_i_plus_half_j(velocityGrid, i - 1, j) *
           velocityGrid.getVelocity_u_i_plus_half_j(i - 1, j));
    } else {
        // TODO: boundary conditions are kinda sus ngl
        uDirectionNumerator = hBar_i_plus_half_j(velocityGrid, i, j) *
                              velocityGrid.getVelocity_u_i_plus_half_j(i, j);
    }

    // Boundary conditions: top boundary
    float wDirectionNumerator = 0.0F;
    if (j > 0) {
        wDirectionNumerator =
          (hBar_i_j_plus_half(velocityGrid, i, j) *
           velocityGrid.getVelocity_w_i_j_plus_half(i, j)) -
          (hBar_i_j_plus_half(velocityGrid, i, j - 1) *
           velocityGrid.getVelocity_w_i_j_plus_half(i, j - 1));
    } else {
        wDirectionNumerator = hBar_i_j_plus_half(velocityGrid, i, j) *
                              velocityGrid.getVelocity_w_i_j_plus_half(i, j);
    }

    constexpr float invDeltaX = 1.0F / deltaX;

    const float delH_delT =
      -(uDirectionNumerator + wDirectionNumerator) * invDeltaX;

    return delH_delT * deltaT;
}

linalg::aliases::float2 WaterSimulation::calcVelocityChangeIntegration(
  size_t i, size_t j, const HeightGrid<numRows, numCols>& heightGrid,
  const linalg::aliases::float3& accelExt) const
{
    const float eta_ij = heightGrid.getEta(i, j);
    constexpr float g_over_deltaX =
      -Physics::gravitationalAccelerationMagnitude / deltaX;

    // Boundary conditions:
    // right boundary
    float eta_i_plus_1_j = eta_ij; // current cell (reflective)
    if (i + 1 < numRows) {
        eta_i_plus_1_j = heightGrid.getEta(i + 1, j);
    }

    // Boundary conditions:
    // bottom boundary
    float eta_i_j_plus_1 = eta_ij; // current cell (reflective)
    if (j + 1 < numCols) {
        eta_i_j_plus_1 = heightGrid.getEta(i, j + 1);
    }

    const float delta_u_i_plus_half_j =
      (g_over_deltaX * (eta_i_plus_1_j - eta_ij) + accelExt.x) * deltaT;
    const float delta_w_i_j_plus_half =
      (g_over_deltaX * (eta_i_j_plus_1 - eta_ij) + accelExt.z) * deltaT;

    return {delta_u_i_plus_half_j, delta_w_i_j_plus_half};
}

// std::unique_ptr<
//   StaggeredGrid<WaterSimulation::numRows, WaterSimulation::numCols>>
void WaterSimulation::advectVelocities()
{
    // Boundary conditions:
    // The paper in 2.1.4 suggests ignoring the velocities on the boundary in
    // advection
    // TODO: does this work???
    static constexpr size_t minI = 1;
    static constexpr size_t maxI = numRows - 1;
    static_assert(minI < maxI);
    static constexpr size_t minJ = 1;
    static constexpr size_t maxJ = numCols - 1;
    static_assert(minJ < maxJ);

    // Advect u
    for (size_t i = minI; i < maxI; ++i) {
        for (size_t j = minJ; j < maxJ + 1; ++j) {
            // pos of u-velocity component
            linalg::aliases::float2 pos = {(static_cast<float>(j) - 0.5F) *
                                             deltaX,
                                           static_cast<float>(i) * deltaX};

            // interpolate the w-velocity at this position to get a full
            // velocity vector. Grid coordinates for w-field interpolation
            linalg::aliases::float2 w_pos_grid = {pos.x / deltaX,
                                                  (pos.y / deltaX) - 0.5F};
            const float w_interp = interpolate<numRows + 1, numCols>(
              velocityGrid.getWVelocities(), w_pos_grid);

            linalg::aliases::float2 vel = {
              velocityGrid.getVelocity_u_i_plus_half_j(i, j), w_interp};

            // trace back in time
            linalg::aliases::float2 departure_pos = pos - vel * deltaT;

            // sample old u-velocity field at departure point.
            // convert departure point in to a u-field grid coords
            linalg::aliases::float2 u_pos_grid = {
              (departure_pos.x / deltaX) + 0.5F, departure_pos.y / deltaX};
            const float new_u = interpolate<numRows, numCols + 1>(
              velocityGrid.getUVelocities(), u_pos_grid);

            velocityGrid.setVelocity_u_i_plus_half_j(i, j, new_u);
        }
    }

    // Advect w
    for (size_t i = minI; i < maxI + 1; ++i) {
        for (size_t j = minJ; j < maxJ; ++j) {
            linalg::aliases::float2 pos = {static_cast<float>(j) * deltaX,
                                           (static_cast<float>(i) - 0.5F) *
                                             deltaX};

            linalg::aliases::float2 u_pos_grid = {(pos.x / deltaX) - 0.5F,
                                                  pos.y / deltaX};
            const float u_interp = interpolate<numRows, numCols + 1>(
              velocityGrid.getUVelocities(), u_pos_grid);

            linalg::aliases::float2 vel = {
              u_interp, velocityGrid.getVelocity_w_i_j_plus_half(i, j)};

            linalg::aliases::float2 departure_pos = pos - vel * deltaT;

            linalg::aliases::float2 w_pos_grid = {
              departure_pos.x / deltaX, (departure_pos.y / deltaX) + 0.5F};
            const float new_w = interpolate<numRows + 1, numCols>(
              velocityGrid.getWVelocities(), w_pos_grid);

            velocityGrid.setVelocity_w_i_j_plus_half(i, j, new_w);
        }
    }
}

bool WaterSimulation::isPositionInWater(
  const linalg::aliases::float3& pos,
  const HeightGrid<WaterSimulation::numRows, WaterSimulation::numCols>& heights)
{
    const bool withinGrid = (pos.x >= bottomLeftCornerWorldPos_xz.x) &&
                            (pos.z >= bottomLeftCornerWorldPos_xz.y) &&
                            (pos.x <= topRightCornerWorldPos_xz.x) &&
                            (pos.z <= topRightCornerWorldPos_xz.y);

    if (!withinGrid) {
        return false;
    }

    const float waterHeight = interpolate<numRows, numCols>(
      heights.getWaterHeights(),
      linalg::aliases::float2{
        (pos.x - bottomLeftCornerWorldPos_xz.x) / cellSize,
        (pos.z - bottomLeftCornerWorldPos_xz.y) / cellSize});

    return (pos.y <= waterHeight);
}

void WaterSimulation::updateFluidWithTriangle(
  const float areaOfTriangle, const linalg::aliases::float3& positionOfCentroid,
  const linalg::aliases::float3& velocityOfCentroid,
  const linalg::aliases::float3& relativeVelocityOfCentroidWRTFluid,
  const linalg::aliases::float3& normalOfCentroid,
  HeightGrid<WaterSimulation::numRows, WaterSimulation::numCols>& heights)
{
    // TODO: unsure if this is meant to be the length
    // TODO: would definitely be better to use a vector with a 0 in the y pos
    // instead of this potential catastrophic cancellation??
    const float horizontalSpeed = linalg::length(
      velocityOfCentroid - velocityOfCentroid.y * upDirection_yHat);

    const float numSubstepsLowerBound = std::floor(
      (horizontalSpeed * (Physics::WaterSim::deltaT / deltaX)) + 0.5F);
    uint32_t numSubsteps =
      static_cast<uint32_t>(std::max(1.0F, numSubstepsLowerBound));

    const float displacementVolume =
      linalg::dot(normalOfCentroid, relativeVelocityOfCentroidWRTFluid) *
      areaOfTriangle * deltaT;

    const float displacementVolumePerSubstep =
      displacementVolume / static_cast<float>(numSubsteps);

    const float heightSign = (normalOfCentroid.y > 0) ? 1.0F : -1.0F;

    for (uint32_t q = 0; q < numSubsteps; ++q) {
        // for (uint32_t q = 0; q < 100; ++q) {
        // P_s
        const linalg::aliases::float3 currentPos =
          positionOfCentroid + velocityOfCentroid * static_cast<float>(q) *
                                 deltaT / static_cast<float>(numSubsteps);

        const auto [i, j] = getClosestGridPoint(currentPos.x, currentPos.z);

        if (i < 0 || i >= static_cast<int>(numRows) || j < 0 ||
            j >= static_cast<int>(numCols))
        {
            continue;
        }

        const float waterSurfaceHeight = heights.getEta(i, j);
        const float depth = waterSurfaceHeight - currentPos.y;

        if (depth > 0) {
            const float decay = std::expf(-decayRate_SolidsToFluids * depth);

            const float heightChange =
              Cdisplacement_SolidsToFluids * decay *
              (displacementVolumePerSubstep / (deltaX * deltaX));

            heights.setWaterHeight(
              i, j, heights.getWaterHeight(i, j) + heightChange, maxDepth);

            const float velCoeffUpperBound =
              decay * Cadapt_SolidsToFluids * (depth / heights.getEta(i, j)) *
              heightSign * (deltaT / (deltaX * deltaX)) * areaOfTriangle;

            const float velCoeff = std::min(1.0F, velCoeffUpperBound);

            const float targetU = velocityOfCentroid.x;
            const float currentU =
              velocityGrid.getVelocity_u_i_plus_half_j(i, j);
            const float velU_change = velCoeff * (targetU - currentU);
            velocityGrid.setVelocity_u_i_plus_half_j(i, j,
                                                     currentU + velU_change);

            const float targetW = velocityOfCentroid.z;
            const float currentW =
              velocityGrid.getVelocity_w_i_j_plus_half(i, j);
            const float velW_change = velCoeff * (targetW - currentW);
            velocityGrid.setVelocity_w_i_j_plus_half(i, j,
                                                     currentW + velW_change);
        }
    }
}

linalg::aliases::float3 WaterSimulation::computeFluidForceOnTriangle(
  const SubTriangle& subTriangle,
  const linalg::aliases::float3& triangleVelocity,
  const HeightGrid<numRows, numCols>& heights) const
{
    const linalg::aliases::float3& pos = subTriangle.centroid;
    const linalg::aliases::float3& normal = subTriangle.normal;
    const float area = subTriangle.area;

    if (!isPositionInWater(pos, heights)) {
        return {0.0F, 0.0F, 0.0F};
    }

    const auto [i, j] = getClosestGridPoint(pos.x, pos.z);
    const float waterHeight = heights.getWaterHeight(i, j);
    const float depth = waterHeight - pos.y;

    if (depth <= 0) {
        return {0.0F, 0.0F, 0.0F};
    }

    linalg::aliases::float3 totalForce = {0.0F, 0.0F, 0.0F};

    constexpr float fluidDensity = 1000.0F;
    constexpr float forceScale =
      Physics::WaterSim::deltaT / Physics::RigidBody::deltaT;

    // buoyancy force (archimedes principle)
    // only apply buoyancy to upward-facing component of surfaces
    if (normal.y > 0) {
        const float maxBuoyancyDepth = std::min(depth, 0.2F); // TODO: what cap?
        const float submergedVolume = maxBuoyancyDepth * area * normal.y;

        linalg::aliases::float3 buoyancyForce =
          fluidDensity * Physics::gravitationalAccelerationMagnitude *
          submergedVolume * upDirection_yHat * forceScale;

        // damping
        constexpr float buoyancyDampingCoeff = 0.1F;
        float upwardVelocity = linalg::dot(triangleVelocity, upDirection_yHat);
        if (upwardVelocity > 0) {
            float dampingForceMagnitude = buoyancyDampingCoeff * fluidDensity *
                                          (area * normal.y) * upwardVelocity *
                                          forceScale;
            buoyancyForce.y -= dampingForceMagnitude;
        }

        totalForce += buoyancyForce;
    }

    const linalg::aliases::float3 fluidVelocity =
      getFluidVelocityAtPosition(pos);
    const linalg::aliases::float3 relativeVelocity =
      triangleVelocity - fluidVelocity;
    const float relativeSpeed = linalg::length(relativeVelocity);

    // damping force (broken)
    // constexpr float dampingCoefficient = 1.0F;
    // const linalg::aliases::float3 dampingForce =
    //   -dampingCoefficient * fluidDensity * area * relativeVelocity;
    // totalForce += dampingForce;
    // Other damping force...maybe this works?
    // constexpr float viscousDampingCoeff = 0.5F;
    // const linalg::aliases::float3 viscousForce = -viscousDampingCoeff *
    //                                              fluidDensity * area *
    //                                              relativeVelocity *
    //                                              forceScale;
    // totalForce += viscousForce;

    // drag and lift
    if (relativeSpeed > 1e-4F) {
        const linalg::aliases::float3 flowDirection =
          relativeVelocity / relativeSpeed;
        const float normalDotVelocity = linalg::dot(normal, relativeVelocity);

        if (normalDotVelocity >= 0.0F) {
            const float normalDotFlow = linalg::dot(normal, flowDirection);
            constexpr float omega = 0.5F;
            const float effectiveArea =
              (normalDotFlow < 0.0F)
                ? 0.0F
                : area * (normalDotFlow * omega + (1.0F - omega));

            // drag force
            constexpr float dragCoefficient = 0.01F; // paper sec. 3.1
            const linalg::aliases::float3 dragForce =
              -0.5F * fluidDensity * dragCoefficient * effectiveArea *
              relativeSpeed * relativeVelocity * forceScale;
            totalForce += dragForce;

            // lift force
            const linalg::aliases::float3 crossProduct =
              linalg::cross(normal, flowDirection);
            const float crossLength = linalg::length(crossProduct);

            if (crossLength > 1e-6F) {
                constexpr float liftCoefficient = 0.005F; // paper sec. 3.1
                const linalg::aliases::float3 liftDirection =
                  crossProduct / crossLength;
                const linalg::aliases::float3 liftForce =
                  0.5F * fluidDensity * liftCoefficient * effectiveArea *
                  relativeSpeed * relativeSpeed * liftDirection * forceScale;
                totalForce += liftForce;
            }

            //                const float pressureCoefficient = 0.5F;
            //                const linalg::aliases::float3 pressureForce =
            //                  -pressureCoefficient * fluidDensity *
            //                  relativeSpeed * relativeSpeed * normalDotFlow *
            //                  normal;
            //                totalForce += pressureForce;
        }
    }

    // force limiting
    const float maxForcePerTriangle = 0.1F;
    const float forceLength = linalg::length(totalForce);
    if (forceLength > maxForcePerTriangle) {
        totalForce = (totalForce / forceLength) * maxForcePerTriangle;
    }

    // Last resort: bad idea
    // totalForce *= Physics::WaterSim::deltaT;

    return totalForce;
}

// in the 2D axis (considers height to be 0)
linalg::aliases::float3 WaterSimulation::getFluidVelocityAtPosition(
  const linalg::aliases::float3& worldPos) const
{
    const float gridX = (worldPos.x - bottomLeftCornerWorldPos_xz.x) / cellSize;
    const float gridZ = (worldPos.z - bottomLeftCornerWorldPos_xz.y) / cellSize;

    const float u = interpolate<numRows, numCols + 1>(
      velocityGrid.getUVelocities(),
      linalg::aliases::float2{gridX + 0.5F, gridZ});

    const float w = interpolate<numRows + 1, numCols>(
      velocityGrid.getWVelocities(),
      linalg::aliases::float2{gridX, gridZ + 0.5F});

    return {u, 0.0F, w};
}

void WaterSimulation::coupleWithRigidBodies(
  std::vector<RigidBodyData>& rigidBodies,
  HeightGrid<WaterSimulation::numRows, WaterSimulation::numCols>& heights)
{
    // TODO: Do this with area < kappa delta x^2 bound as suggested by the paper
    constexpr int subdivisions = 1;

    // TODO: Add AABB to further optimie this loop
    for (auto& rigidBody : rigidBodies) {
        rigidBody.invalidateTriangleCache();
        const std::vector<Triangle>& triangles = rigidBody.getTriangles();

        for (const Triangle& triangle : triangles) {
            std::vector<SubTriangle> subTriangles =
              triangle.subdivide(subdivisions);

            for (const SubTriangle& subTriangle : subTriangles) {
                if (!isPositionInWater(subTriangle.centroid, heights)) {
                    continue;
                }

                linalg::aliases::float3 triangleVelocity =
                  rigidBody.getPointVelocity(subTriangle.centroid);

                linalg::aliases::float3 fluidVelocity =
                  getFluidVelocityAtPosition(subTriangle.centroid);

                linalg::aliases::float3 relativeVelocity =
                  triangleVelocity - fluidVelocity;

                linalg::aliases::float3 fluidForce =
                  computeFluidForceOnTriangle(subTriangle, triangleVelocity,
                                              heights);

                rigidBody.applyForce(fluidForce, subTriangle.centroid);

                updateFluidWithTriangle(subTriangle.area, subTriangle.centroid,
                                        triangleVelocity, relativeVelocity,
                                        subTriangle.normal, heights);
            }
        }
    }
}
