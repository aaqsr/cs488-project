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
#include <string>

// using Physics::WaterSim::deltaT;

namespace
{

using Real = WaterSimulation::Real;
using Real2 = WaterSimulation::Real2;
using Real3 = WaterSimulation::Real3;

template <size_t rows, size_t cols>
[[nodiscard]]
Real interpolate(const std::array<float, rows * cols>& field, const Real2& pos)
{
    // Boundary Conditions:
    auto getFieldValue = [&field](int i, int j) -> Real {
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

    const Real tx = pos.x - static_cast<float>(i0);
    const Real ty = pos.y - static_cast<float>(j0);

    const Real v00 = getFieldValue(i0, j0);
    const Real v10 = getFieldValue(i1, j0);
    const Real v01 = getFieldValue(i0, j1);
    const Real v11 = getFieldValue(i1, j1);

    // linear interp in x
    const Real lerpX1 = v00 + ((v10 - v00) * tx);
    const Real lerpX2 = v01 + ((v11 - v01) * tx);

    // linear interp in y
    return lerpX1 + ((lerpX2 - lerpX1) * ty);
}

// Real calculateAdjustedHeight(
//   size_t i, size_t j,
//   const HeightGrid<WaterSimulation::numRows, WaterSimulation::numCols>&
//     heightGrid)
// {
//     // Stability enhancement from section 2.1.5
//     // hadj = max(0, (h_i+1,j + h_i-1,j + h_i,j+1 + h_i,j-1)/4 - havgmax)
//     // havgmax = β * Δx / (g * Δt)
//
//     constexpr Real beta = 2.0F;
//     const Real h_avg_max =
//       beta * WaterSimulation::deltaX /
//       (Physics::gravitationalAccelerationMagnitude *
//       Physics::WaterSim::deltaT);
//
//     // Calculate average height of neighbours
//     Real neighbourSum = 0.0F;
//     int neighbourCount = 0;
//
//     const std::array<std::pair<int, int>, 4> neighbourIndices = {
//       {
//        {static_cast<int>(i) + 1, static_cast<int>(j)}, // right
//         {static_cast<int>(i) - 1, static_cast<int>(j)}, // left
//         {static_cast<int>(i), static_cast<int>(j) + 1}, // bottom
//         {static_cast<int>(i), static_cast<int>(j) - 1}  // top
//       }
//     };
//
//     for (const auto& [ni, nj] : neighbourIndices) {
//         // for boundary cells, use available neighbours
//         if (ni >= 0 && ni < static_cast<int>(WaterSimulation::numRows) &&
//             nj >= 0 && nj < static_cast<int>(WaterSimulation::numCols))
//         {
//             neighbourSum += heightGrid.getWaterHeight(ni, nj);
//             neighbourCount++;
//         }
//     }
//
//     Real avgNeighbourHeight =
//       (neighbourCount > 0) ? neighbourSum /
//       static_cast<Real>(neighbourCount)
//                            : heightGrid.getWaterHeight(i, j);
//
//     // Calculate hadj
//     Real h_adj = std::max(0.0F, avgNeighbourHeight - h_avg_max);
//
//     // TODO : damping. Good idea?
//     constexpr Real dampingFactor = 0.8F;
//     Real adjustedHeight =
//       heightGrid.getWaterHeight(i, j) - (dampingFactor * h_adj);
//     return std::max(0.0F, adjustedHeight);
//
//     // adjusted height: h_bar - h_adj
//     // return heightGrid.getWaterHeight(i, j) - h_adj;
// }

std::pair<int, int> getClosestGridPoint(Real x, float z)
{
    Real gridX = (x - WaterSimulation::bottomLeftCornerWorldPos_xz.x) /
                 Physics::WaterSim::cellSize;
    Real gridZ = (z - WaterSimulation::bottomLeftCornerWorldPos_xz.y) /
                 Physics::WaterSim::cellSize;

    int i = std::clamp(static_cast<int>(std::round(gridZ)), 0,
                       static_cast<int>(WaterSimulation::numRows - 1));
    int j = std::clamp(static_cast<int>(std::round(gridX)), 0,
                       static_cast<int>(WaterSimulation::numCols - 1));

    return {i, j};
}

} // namespace

WaterSimulation::WaterSimulation() = default;

void WaterSimulation::setInitConditions(
  HeightGrid<WaterSimulation::numRows, WaterSimulation::numCols>& heightGrid)
{
    constexpr static auto fn = [](Real x, float y) {
        // Gaussian hump function
        const Real sigma = 0.25F;    // width of the hump
        const Real amplitude = 0.4F; // height of the hump
        Real r_squared = (x * x) + (y * y);
        return amplitude * std::exp(-r_squared / (2.0F * sigma * sigma));
    };

    for (size_t i = 0; i < WaterSimulation::numRows; ++i) {
        for (size_t j = 0; j < WaterSimulation::numCols; ++j) {
            if (((i >= WaterSimulation::numRows / 4) &&
                 (i < (3 * WaterSimulation::numRows) / 4)) &&
                ((j >= WaterSimulation::numCols / 4) &&
                 (j < (3 * WaterSimulation::numCols) / 4)))
            {
                Real x = (static_cast<float>(i) -
                          (WaterSimulation::numRows * 0.5F) + 0.5F) *
                         WaterSimulation::cellSize;
                Real y = (static_cast<float>(j) -
                          (WaterSimulation::numCols * 0.5F) + 0.5F) *
                         WaterSimulation::cellSize;

                // add the hump to the base level of 1.0
                heightGrid.setWaterHeight(i, j, 1.0F + fn(x, y));
            } else {
                heightGrid.setWaterHeight(i, j, 1.0F);
            }
        }
    }
}

void WaterSimulation::update(HeightGrid<numRows, numCols>& newHeightGrid,
                             const HeightGrid<numRows, numCols>& prevHeightGrid)
{
    const int requiredSubSteps = calculateOptimalSubSteps(prevHeightGrid);
    const Real subDeltaT =
      Physics::WaterSim::deltaT / static_cast<Real>(requiredSubSteps);

    // if start of new physics step, init. sub-stepping
    if (!subStepInProgress) {
        for (size_t i = 0; i < numRows; ++i) {
            for (size_t j = 0; j < numCols; ++j) {
                newHeightGrid.setWaterHeight(
                  i, j, prevHeightGrid.getWaterHeight(i, j));
            }
        }
        subStepInProgress = true;
        currentSubStep = 0;
        totalSubSteps = requiredSubSteps;
        accumulatedTime = 0.0;
    }

    for (int step = 0; step < totalSubSteps; ++step) {
        // substeps after first, newHeightGrid becomes the prev state
        if (step == 0) {
            performSingleSubStep(newHeightGrid, prevHeightGrid, subDeltaT);
        } else {
            // in-place update
            performSingleSubStep(newHeightGrid, newHeightGrid, subDeltaT);
        }
        accumulatedTime += subDeltaT;
    }

    const Real timeError = Physics::WaterSim::deltaT - accumulatedTime;
    if (std::abs(timeError) > 1e-8) {
        performSingleSubStep(newHeightGrid, newHeightGrid, timeError);
    }

    // reset
    subStepInProgress = false;
    currentSubStep = 0;
    accumulatedTime = 0.0;

    // performStabilityCheck(newHeightGrid);

    {
        // corner boundary condition to prevent corner (0, 0) from getting
        // stuck to init height throughout simulation
        // TODO: THIS IS A HACK. Impl properly?
        // newHeightGrid.setWaterHeight(0, 0, newHeightGrid.getWaterHeight(0,
        // 1));
    }
}

void WaterSimulation::performSingleSubStep(
  HeightGrid<numRows, numCols>& newHeightGrid,
  const HeightGrid<numRows, numCols>& prevHeightGrid, Real subDeltaT)
{
    // did override it (TODO: holy hell this is no longer evil)
    WaterSimulation::adaptiveDeltaT = subDeltaT;

    advectVelocities();

    // Create temporary storage for intermediate height calculations
    // We'll calculate all new heights first, then update the grid
    static thread_local std::vector<Real> tempHeights(numRows * numCols);

    // Update heights - calculate all new values first
    for (size_t i = 0; i < numRows; ++i) {
        for (size_t j = 0; j < numCols; ++j) {
            const Real heightChange =
              calcHeightChangeIntegral(i, j, prevHeightGrid);
            const Real newHeight =
              prevHeightGrid.getWaterHeight(i, j) + heightChange;
            tempHeights[(i * numCols) + j] =
              std::clamp(newHeight, static_cast<Real>(0.0),
                         static_cast<Real>(Physics::WaterSim::maxDepth));
        }
    }

    // Now update the actual grid
    for (size_t i = 0; i < numRows; ++i) {
        for (size_t j = 0; j < numCols; ++j) {
            newHeightGrid.setWaterHeight(i, j, tempHeights[(i * numCols) + j]);
        }
    }

    // Update velocities
    for (size_t i = 0; i < numRows; ++i) {
        for (size_t j = 0; j < numCols; ++j) {
            const auto& [deltaU, deltaW] =
              calcVelocityChangeIntegration(i, j, newHeightGrid);

            const Real newU =
              velocityGrid.getVelocity_u_i_plus_half_j(i, j) + deltaU;
            velocityGrid.setVelocity_u_i_plus_half_j(
              i, j, velocityComponentDissipationConstant * newU);

            const Real newW =
              velocityGrid.getVelocity_w_i_j_plus_half(i, j) + deltaW;
            velocityGrid.setVelocity_w_i_j_plus_half(
              i, j, velocityComponentDissipationConstant * newW);
        }
    }

    // Apply boundary conditions
    for (size_t j = 0; j < numCols; ++j) {
        velocityGrid.setVelocity_w_i_j_plus_half(0, j, 0.0F);
        velocityGrid.setVelocity_w_i_j_plus_half(numRows, j, 0.0F);
    }

    for (size_t i = 0; i < numRows; ++i) {
        velocityGrid.setVelocity_u_i_plus_half_j(i, 0, 0.0F);
        velocityGrid.setVelocity_u_i_plus_half_j(i, numCols, 0.0F);
    }
}

int WaterSimulation::calculateOptimalSubSteps(
  const HeightGrid<numRows, numCols>& heightGrid) const
{
    Real maxWaveSpeed = 0.0F;
    Real maxAdvectionSpeed = 0.0F;

    // Find maximum wave and advection speeds
    for (size_t i = 0; i < numRows; ++i) {
        for (size_t j = 0; j < numCols; ++j) {
            Real h = heightGrid.getWaterHeight(i, j);
            if (h > 1e-6F) {
                Real waveSpeed =
                  std::sqrt(Physics::gravitationalAccelerationMagnitude * h);
                maxWaveSpeed = std::max(maxWaveSpeed, waveSpeed);
            }

            // Check velocity components
            if (j < numCols) {
                Real u =
                  std::abs(velocityGrid.getVelocity_u_i_plus_half_j(i, j));
                maxAdvectionSpeed = std::max(maxAdvectionSpeed, u);
            }
            if (i < numRows) {
                Real w =
                  std::abs(velocityGrid.getVelocity_w_i_j_plus_half(i, j));
                maxAdvectionSpeed = std::max(maxAdvectionSpeed, w);
            }
        }
    }

    Real maxSpeed = maxWaveSpeed + maxAdvectionSpeed;
    if (maxSpeed < 1e-6F) {
        return 1; // Minimum sub-steps
    }

    // CFL condition: dt <= C * dx / max_speed
    // Use safety factor of 0.8
    constexpr Real cflSafetyFactor = 0.8F;
    Real maxAllowedDt = cflSafetyFactor * deltaX / maxSpeed;

    // Calculate required sub-steps to satisfy CFL
    int requiredSubSteps =
      static_cast<int>(std::ceil(Physics::WaterSim::deltaT / maxAllowedDt));

    // Clamp to reasonable bounds
    return std::clamp(requiredSubSteps, 1, 20);
}

Real WaterSimulation::calcHeightChangeIntegral(
  size_t i, size_t j, const HeightGrid<numRows, numCols>& heightGrid) const
{
    // Both evaluated in the upwind direction
    // TODO: why capturing one but not the other??
    // const auto hBar_i_plus_half_j =
    //   [&heightGrid](const decltype(velocityGrid)& grid, size_t i,
    //                 size_t j) -> Real {
    //     // Boundary conditions:
    //     // if at right edge, use current cell height
    //     if (i + 1 >= numRows) {
    //         return heightGrid.getWaterHeight(i, j);
    //     }
    //
    //     return (grid.getVelocity_u_i_plus_half_j(i, j) <= 0)
    //              ? heightGrid.getWaterHeight(i + 1, j)
    //              : heightGrid.getWaterHeight(i, j);
    // };
    // const auto hBar_i_j_plus_half =
    //   [&heightGrid](const decltype(velocityGrid)& grid, size_t i,
    //                 size_t j) -> Real {
    //     if (j + 1 >= numCols) {
    //         return heightGrid.getWaterHeight(i, j);
    //     }
    //     return (grid.getVelocity_w_i_j_plus_half(i, j) <= 0)
    //              ? heightGrid.getWaterHeight(i, j + 1)
    //              : heightGrid.getWaterHeight(i, j);
    // };

    // Stability improvement in section 2.1.5 final paragraph
    const auto getStabilisedHeight = [&heightGrid](size_t ii,
                                                   size_t jj) -> Real {
        // havgmax
        constexpr Real beta = 0.5F;
        const Real h_ij = heightGrid.getWaterHeight(ii, jj);
        const Real havgmax = beta * WaterSimulation::deltaX /
                             (Physics::gravitationalAccelerationMagnitude *
                              WaterSimulation::adaptiveDeltaT);

        // get neighbours
        Real neighbourSum = 0.0F;
        int count = 0;
        const std::array<std::pair<int, int>, 4> neighbourIndicies = {
          {{static_cast<int>(ii) + 1, static_cast<int>(jj)},
           {static_cast<int>(ii) - 1, static_cast<int>(jj)},
           {static_cast<int>(ii), static_cast<int>(jj) + 1},
           {static_cast<int>(ii), static_cast<int>(jj) - 1}}
        };

        for (const auto& [ni, nj] : neighbourIndicies) {
            if (ni >= 0 && ni < static_cast<int>(numRows) && nj >= 0 &&
                nj < static_cast<int>(numCols))
            {
                neighbourSum += heightGrid.getWaterHeight(ni, nj);
                count++;
            } else {
                neighbourSum += h_ij;
                count++;
            }
        }

        Real avgHeight =
          (count > 0) ? neighbourSum / static_cast<Real>(count) : h_ij;
        Real hadj = std::max(static_cast<Real>(0.0), avgHeight - havgmax);

        return std::max(static_cast<Real>(0.0), h_ij - hadj);
    };

    // const auto getStabilisedHeight = [&heightGrid](size_t ii,
    //                                                size_t jj) -> Real {
    //     return heightGrid.getWaterHeight(ii, jj);
    // };

    // heights in upwind evaluation
    const auto hBar_i_plus_half_j =
      [&getStabilisedHeight](
        const StaggeredVelocityGrid<numRows, numCols>& velocityGrid, size_t ii,
        size_t jj) -> Real {
        // boundary condition: if at right edge, use current cell
        if (ii + 1 >= numRows) {
            return getStabilisedHeight(ii, jj);
        }
        Real u_vel = velocityGrid.getVelocity_u_i_plus_half_j(ii, jj);
        return (u_vel <= 0.0F) ? getStabilisedHeight(ii + 1, jj)
                               : getStabilisedHeight(ii, jj);
    };
    const auto hBar_i_j_plus_half =
      [&getStabilisedHeight](
        const StaggeredVelocityGrid<numRows, numCols>& velocityGrid, size_t ii,
        size_t jj) -> Real {
        // boundary condition: if at bottom edge, use current cell
        if (jj + 1 >= numCols) {
            return getStabilisedHeight(ii, jj);
        }
        Real w_vel = velocityGrid.getVelocity_w_i_j_plus_half(ii, jj);
        return (w_vel <= 0.0F) ? getStabilisedHeight(ii, jj + 1)
                               : getStabilisedHeight(ii, jj);
    };

    // Boundary conditions:
    // Real uDirectionFlux = 0.0F;
    // if (i > 0) {
    //     const Real rightFaceFlux =
    //       hBar_i_plus_half_j(velocityGrid, i, j) *
    //       velocityGrid.getVelocity_u_i_plus_half_j(i, j);
    //     const Real leftFaceFlux =
    //       hBar_i_plus_half_j(velocityGrid, i - 1, j) *
    //       velocityGrid.getVelocity_u_i_plus_half_j(i - 1, j);
    //     uDirectionFlux = rightFaceFlux - leftFaceFlux;
    // } else {
    //     uDirectionFlux = hBar_i_plus_half_j(velocityGrid, i, j) *
    //                      velocityGrid.getVelocity_u_i_plus_half_j(i, j);
    // }
    // Real wDirectionFlux = 0.0F;
    // if (j > 0) {
    //     const Real bottomFaceFlux =
    //       hBar_i_j_plus_half(velocityGrid, i, j) *
    //       velocityGrid.getVelocity_w_i_j_plus_half(i, j);
    //     const Real topFaceFlux =
    //       hBar_i_j_plus_half(velocityGrid, i, j - 1) *
    //       velocityGrid.getVelocity_w_i_j_plus_half(i, j - 1);
    //     wDirectionFlux = bottomFaceFlux - topFaceFlux;
    // } else {
    //     wDirectionFlux = hBar_i_j_plus_half(velocityGrid, i, j) *
    //                      velocityGrid.getVelocity_w_i_j_plus_half(i, j);
    // }

    // boundary conditions v2.0: (u direction)
    // we calculate flux differences with stability limiting
    // Real uDirectionFlux = 0.0F;
    // Real wDirectionFlux = 0.0F;
    // if (i > 0) {
    //     Real rightFaceFlux = hBar_i_plus_half_j(i, j) *
    //                           velocityGrid.getVelocity_u_i_plus_half_j(i, j);
    //     Real leftFaceFlux = hBar_i_plus_half_j(i - 1, j) *
    //                          velocityGrid.getVelocity_u_i_plus_half_j(i - 1,
    //                          j);
    //     uDirectionFlux = rightFaceFlux - leftFaceFlux;
    // } else {
    //     // left boundary: only outflow
    //     uDirectionFlux =
    //       hBar_i_plus_half_j(i, j) *
    //       std::max(0.0F, velocityGrid.getVelocity_u_i_plus_half_j(i, j));
    // }
    //
    // // boundary conditions: (w direction)
    // if (j > 0) {
    //     Real bottomFaceFlux = hBar_i_j_plus_half(i, j) *
    //                            velocityGrid.getVelocity_w_i_j_plus_half(i,
    //                            j);
    //     Real topFaceFlux = hBar_i_j_plus_half(i, j - 1) *
    //                         velocityGrid.getVelocity_w_i_j_plus_half(i, j -
    //                         1);
    //     wDirectionFlux = bottomFaceFlux - topFaceFlux;
    // } else {
    //     // top boundary: only outflow
    //     wDirectionFlux =
    //       hBar_i_j_plus_half(i, j) *
    //       std::max(0.0F, velocityGrid.getVelocity_w_i_j_plus_half(i, j));
    // }

    Real uDirectionFlux = 0.0F;
    const Real rightFaceFlux = hBar_i_plus_half_j(velocityGrid, i, j) *
                               velocityGrid.getVelocity_u_i_plus_half_j(i, j);
    Real leftFaceFlux = 0.0F;
    if (i > 0) {
        leftFaceFlux = hBar_i_plus_half_j(velocityGrid, i - 1, j) *
                       velocityGrid.getVelocity_u_i_plus_half_j(i - 1, j);
    }
    uDirectionFlux = rightFaceFlux - leftFaceFlux;

    Real wDirectionFlux = 0.0F;
    const Real bottomFaceFlux = hBar_i_j_plus_half(velocityGrid, i, j) *
                                velocityGrid.getVelocity_w_i_j_plus_half(i, j);
    Real topFaceFlux = 0.0F;
    if (j > 0) {
        topFaceFlux = hBar_i_j_plus_half(velocityGrid, i, j - 1) *
                      velocityGrid.getVelocity_w_i_j_plus_half(i, j - 1);
    }
    wDirectionFlux = bottomFaceFlux - topFaceFlux;

    constexpr Real invDeltaX = 1.0F / deltaX;

    const Real totalFluxDivergence =
      (uDirectionFlux + wDirectionFlux) * invDeltaX;
    Real heightChange = -totalFluxDivergence * WaterSimulation::adaptiveDeltaT;

    //
    // again apply flux limiting to prevent extreme changes
    // const Real currentHeight = heightGrid.getWaterHeight(i, j);
    // const Real maxAllowedChange = 0.5F * currentHeight; // limit to 50% /
    // step const Real heightChange = delH_delT * Physics::WaterSim::deltaT;
    //
    // return std::clamp(heightChange, -maxAllowedChange, maxAllowedChange);

    // limit max height change / step
    Real currentHeight = heightGrid.getWaterHeight(i, j);
    if (currentHeight > 1e-6F) {
        Real maxAllowedChangeRatio = 0.25F;
        Real maxAllowedChange = maxAllowedChangeRatio * currentHeight;
        heightChange =
          std::clamp(heightChange, -maxAllowedChange, maxAllowedChange);
    }

    return heightChange;
}

void WaterSimulation::updateHeightsSemiImplicit(
  HeightGrid<numRows, numCols>& newHeightGrid,
  const HeightGrid<numRows, numCols>& prevHeightGrid)
{
    // Semi-implicit scheme: theta = 0.5 for Crank-Nicolson
    constexpr Real theta = 0.5F;
    constexpr int maxIterations = 8;
    constexpr Real tolerance = 1e-5F;

    // Initialize with explicit step
    for (size_t i = 0; i < numRows; ++i) {
        for (size_t j = 0; j < numCols; ++j) {
            const Real explicitChange =
              calcHeightChangeIntegral(i, j, prevHeightGrid);
            const Real newHeight =
              prevHeightGrid.getWaterHeight(i, j) + explicitChange;
            newHeightGrid.setWaterHeight(i, j, newHeight);
        }
    }

    // refine implicit component
    for (int iter = 0; iter < maxIterations; ++iter) {
        HeightGrid<numRows, numCols> tempGrid = newHeightGrid;
        Real maxChange = 0.0F;

        for (size_t i = 0; i < numRows; ++i) {
            for (size_t j = 0; j < numCols; ++j) {
                const Real explicitChange =
                  calcHeightChangeIntegral(i, j, prevHeightGrid);
                const Real implicitChange =
                  calcHeightChangeIntegral(i, j, tempGrid);

                const Real totalChange =
                  ((1.0F - theta) * explicitChange) + (theta * implicitChange);

                const Real newHeight =
                  prevHeightGrid.getWaterHeight(i, j) + totalChange;
                const Real clampedHeight =
                  std::clamp(newHeight, static_cast<Real>(0.0),
                             static_cast<Real>(Physics::WaterSim::maxDepth));
                const Real oldHeight = newHeightGrid.getWaterHeight(i, j);

                newHeightGrid.setWaterHeight(i, j, clampedHeight);

                maxChange =
                  std::max(maxChange, std::abs(clampedHeight - oldHeight));
            }
        }

        if (maxChange < tolerance) {
            break; // converged early
        }
    }
}

Real2 WaterSimulation::calcVelocityChangeIntegration(
  size_t i, size_t j, const HeightGrid<numRows, numCols>& heightGrid,
  const Real3& accelExt) const
{
    const Real eta_ij = heightGrid.getEta(i, j);
    constexpr Real g_over_deltaX =
      -Physics::gravitationalAccelerationMagnitude / deltaX;

    // Boundary conditions:
    // right boundary
    Real eta_i_plus_1_j = eta_ij; // current cell (reflective)
    if (i + 1 < numRows) {
        eta_i_plus_1_j = heightGrid.getEta(i + 1, j);
    }

    // Boundary conditions:
    // bottom boundary
    Real eta_i_j_plus_1 = eta_ij; // current cell (reflective)
    if (j + 1 < numCols) {
        eta_i_j_plus_1 = heightGrid.getEta(i, j + 1);
    }

    const Real delta_u_i_plus_half_j =
      (g_over_deltaX * (eta_i_plus_1_j - eta_ij) + accelExt.x) *
      WaterSimulation::adaptiveDeltaT;
    const Real delta_w_i_j_plus_half =
      (g_over_deltaX * (eta_i_j_plus_1 - eta_ij) + accelExt.z) *
      WaterSimulation::adaptiveDeltaT;

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
            Real2 pos = {(static_cast<Real>(j) - 0.5F) * deltaX,
                         static_cast<Real>(i) * deltaX};

            // interpolate the w-velocity at this position to get a full
            // velocity vector. Grid coordinates for w-field interpolation
            Real2 w_pos_grid = {pos.x / deltaX, (pos.y / deltaX) - 0.5F};
            const Real w_interp = interpolate<numRows + 1, numCols>(
              velocityGrid.getWVelocities(), w_pos_grid);

            Real2 vel = {velocityGrid.getVelocity_u_i_plus_half_j(i, j),
                         w_interp};

            // trace back in time
            Real2 departure_pos = pos - vel * WaterSimulation::adaptiveDeltaT;

            // sample old u-velocity field at departure point.
            // convert departure point in to a u-field grid coords
            Real2 u_pos_grid = {(departure_pos.x / deltaX) + 0.5F,
                                departure_pos.y / deltaX};
            const Real new_u = interpolate<numRows, numCols + 1>(
              velocityGrid.getUVelocities(), u_pos_grid);

            velocityGrid.setVelocity_u_i_plus_half_j(i, j, new_u);
        }
    }

    // Advect w
    for (size_t i = minI; i < maxI + 1; ++i) {
        for (size_t j = minJ; j < maxJ; ++j) {
            Real2 pos = {static_cast<Real>(j) * deltaX,
                         (static_cast<Real>(i) - 0.5F) * deltaX};

            Real2 u_pos_grid = {(pos.x / deltaX) - 0.5F, pos.y / deltaX};
            const Real u_interp = interpolate<numRows, numCols + 1>(
              velocityGrid.getUVelocities(), u_pos_grid);

            Real2 vel = {u_interp,
                         velocityGrid.getVelocity_w_i_j_plus_half(i, j)};

            Real2 departure_pos = pos - vel * WaterSimulation::adaptiveDeltaT;

            Real2 w_pos_grid = {departure_pos.x / deltaX,
                                (departure_pos.y / deltaX) + 0.5F};
            const Real new_w = interpolate<numRows + 1, numCols>(
              velocityGrid.getWVelocities(), w_pos_grid);

            velocityGrid.setVelocity_w_i_j_plus_half(i, j, new_w);
        }
    }
}

void WaterSimulation::performStabilityCheck(
  const HeightGrid<numRows, numCols>& heightGrid)
{
    if ((++stabilityCheckCounter % 100) != 0) {
        return;
    }

    // approximate the current energy
    Real totalEnergy = 0.0F;
    Real maxHeight = 0.0F;
    Real maxVelocity = 0.0F;

    for (size_t i = 0; i < numRows; ++i) {
        for (size_t j = 0; j < numCols; ++j) {
            const Real h = heightGrid.getWaterHeight(i, j);
            maxHeight = std::max(maxHeight, h);

            // potential energy
            totalEnergy +=
              0.5F * Physics::gravitationalAccelerationMagnitude * h * h;

            // kinetic energy (approx)
            if (j < numCols) {
                Real u = velocityGrid.getVelocity_u_i_plus_half_j(i, j);
                maxVelocity = std::max(maxVelocity, std::abs(u));
                totalEnergy += 0.25F * h * u * u;
            }
            if (i < numRows) {
                Real w = velocityGrid.getVelocity_w_i_j_plus_half(i, j);
                maxVelocity = std::max(maxVelocity, std::abs(w));
                totalEnergy += 0.25F * h * w * w;
            }
        }
    }

    // check for energy explosion (instability RED FLAG wee woo wee woo)
    if (stabilityCheckCounter > 100) {
        const Real energyGrowthRatio = totalEnergy / previousTotalEnergy;
        if (energyGrowthRatio > 1.1F) { // TODO: 10% growth is suspicious??
            Logger::GetInstance().log(
              "Warning: Energy growing rapidly, ratio = " +
              std::to_string(energyGrowthRatio));
        }
    }

    previousTotalEnergy = totalEnergy;

    if (stabilityCheckCounter % 500 == 0) {
        Logger::GetInstance().log(
          std::format("Stability check: Max height = {:.4f}, Max velocity = "
                      "{:.4f}, Total energy = {:.2f}",
                      maxHeight, maxVelocity, totalEnergy));
    }
}

bool WaterSimulation::isPositionInWater(
  const Real3& pos,
  const HeightGrid<WaterSimulation::numRows, WaterSimulation::numCols>& heights)
{
    const bool withinGrid = (pos.x >= bottomLeftCornerWorldPos_xz.x) &&
                            (pos.z >= bottomLeftCornerWorldPos_xz.y) &&
                            (pos.x <= topRightCornerWorldPos_xz.x) &&
                            (pos.z <= topRightCornerWorldPos_xz.y);

    if (!withinGrid) {
        return false;
    }

    const Real waterHeight = interpolate<numRows, numCols>(
      heights.getWaterHeights(),
      Real2{(pos.x - bottomLeftCornerWorldPos_xz.x) / cellSize,
            (pos.z - bottomLeftCornerWorldPos_xz.y) / cellSize});

    return (pos.y <= waterHeight);
}

void WaterSimulation::updateFluidWithTriangle(
  const Real areaOfTriangle, const Real3& positionOfCentroid,
  const Real3& velocityOfCentroid,
  const Real3& relativeVelocityOfCentroidWRTFluid,
  const Real3& normalOfCentroid,
  HeightGrid<WaterSimulation::numRows, WaterSimulation::numCols>& heights)
{
    // TODO: unsure if this is meant to be the length
    // TODO: would definitely be better to use a vector with a 0 in the y pos
    // instead of this potential catastrophic cancellation??
    const Real horizontalSpeed = linalg::length(
      velocityOfCentroid - velocityOfCentroid.y * upDirection_yHat);

    const Real numSubstepsLowerBound = std::floor(
      (horizontalSpeed * (Physics::WaterSim::deltaT / deltaX)) + 0.5F);
    uint32_t numSubsteps = static_cast<uint32_t>(
      std::max(static_cast<Real>(1.0), numSubstepsLowerBound));

    const Real displacementVolume =
      linalg::dot(normalOfCentroid, relativeVelocityOfCentroidWRTFluid) *
      areaOfTriangle * Physics::WaterSim::deltaT;

    const Real displacementVolumePerSubstep =
      displacementVolume / static_cast<Real>(numSubsteps);

    const Real heightSign = (normalOfCentroid.y > 0) ? 1.0F : -1.0F;

    for (uint32_t q = 0; q < numSubsteps; ++q) {
        // for (uint32_t q = 0; q < 100; ++q) {
        // P_s
        const Real3 currentPos =
          positionOfCentroid + velocityOfCentroid * static_cast<Real>(q) *
                                 Physics::WaterSim::deltaT /
                                 static_cast<Real>(numSubsteps);

        const auto [i, j] = getClosestGridPoint(currentPos.x, currentPos.z);

        if (i < 0 || i >= static_cast<int>(numRows) || j < 0 ||
            j >= static_cast<int>(numCols))
        {
            continue;
        }

        const Real waterSurfaceHeight = heights.getEta(i, j);
        const Real depth = waterSurfaceHeight - currentPos.y;

        if (depth > 0) {
            const Real decay = std::expf(-decayRate_SolidsToFluids * depth);

            const Real heightChange =
              Cdisplacement_SolidsToFluids * decay *
              (displacementVolumePerSubstep / (deltaX * deltaX));

            heights.setWaterHeight(i, j,
                                   heights.getWaterHeight(i, j) + heightChange);

            const Real velCoeffUpperBound =
              decay * Cadapt_SolidsToFluids * (depth / heights.getEta(i, j)) *
              heightSign * (Physics::WaterSim::deltaT / (deltaX * deltaX)) *
              areaOfTriangle;

            const Real velCoeff =
              std::min(static_cast<Real>(1.0), velCoeffUpperBound);

            const Real targetU = velocityOfCentroid.x;
            const Real currentU =
              velocityGrid.getVelocity_u_i_plus_half_j(i, j);
            const Real velU_change = velCoeff * (targetU - currentU);
            velocityGrid.setVelocity_u_i_plus_half_j(i, j,
                                                     currentU + velU_change);

            const Real targetW = velocityOfCentroid.z;
            const Real currentW =
              velocityGrid.getVelocity_w_i_j_plus_half(i, j);
            const Real velW_change = velCoeff * (targetW - currentW);
            velocityGrid.setVelocity_w_i_j_plus_half(i, j,
                                                     currentW + velW_change);
        }
    }
}

Real3 WaterSimulation::computeFluidForceOnTriangle(
  const SubTriangle& subTriangle, const Real3& triangleVelocity,
  const HeightGrid<numRows, numCols>& heights) const
{
    const Real3& pos = static_cast<Real3>(subTriangle.centroid);
    const Real3& normal = static_cast<Real3>(subTriangle.normal);
    const Real area = subTriangle.area;

    if (!isPositionInWater(pos, heights)) {
        return {0.0F, 0.0F, 0.0F};
    }

    const auto [i, j] = getClosestGridPoint(pos.x, pos.z);
    const Real waterHeight = heights.getWaterHeight(i, j);
    const Real depth = waterHeight - pos.y;

    if (depth <= 0) {
        return {0.0F, 0.0F, 0.0F};
    }

    Real3 totalForce = {0.0F, 0.0F, 0.0F};

    constexpr Real fluidDensity = 1000.0F;
    constexpr Real forceScale =
      Physics::WaterSim::deltaT / Physics::RigidBody::deltaT;

    // time for buoyancy force (archimedes principle) yayyy \o/
    const Real submergedVolume = depth * area * normal.y;

    Real3 buoyancyForce = fluidDensity *
                          (-Physics::gravitationalAccelerationMagnitude) *
                          submergedVolume * upDirection_yHat * forceScale;

    // damping
    // constexpr Real buoyancyDampingCoeff = 0.1F;
    // Real upwardVelocity = linalg::dot(triangleVelocity,
    // upDirection_yHat); if (upwardVelocity > 0) {
    //     Real dampingForceMagnitude = buoyancyDampingCoeff * fluidDensity
    //     *
    //                                  (area * normal.y) * upwardVelocity *
    //                                  forceScale;
    //     buoyancyForce.y -= dampingForceMagnitude;
    // }

    totalForce += buoyancyForce;

    const Real3 fluidVelocity = getFluidVelocityAtPosition(pos);
    const Real3 relativeVelocity = triangleVelocity - fluidVelocity;
    const Real relativeSpeed = linalg::length(relativeVelocity);

    // damping force (broken)
    // constexpr Real dampingCoefficient = 1.0F;
    // const Real3 dampingForce =
    //   -dampingCoefficient * fluidDensity * area * relativeVelocity;
    // totalForce += dampingForce;
    // Other damping force...maybe this works?
    // constexpr Real viscousDampingCoeff = 0.5F;
    // const Real3 viscousForce = -viscousDampingCoeff *
    //                                              fluidDensity * area *
    //                                              relativeVelocity *
    //                                              forceScale;
    // totalForce += viscousForce;

    // drag and lift
    if (relativeSpeed > 1e-6F) {
        const Real3 flowDirection = relativeVelocity / relativeSpeed;
        const Real normalDotVelocity = linalg::dot(normal, relativeVelocity);

        if (normalDotVelocity >= 0.0F) {
            const Real normalDotFlow = linalg::dot(normal, flowDirection);

            constexpr Real omega = 0.9F;

            const Real effectiveArea =
              (normalDotFlow < 0.0F)
                ? 0.0F
                : area * (normalDotFlow * omega + (1.0F - omega));


            // drag force (for cylinder)
            constexpr Real dragCoefficient = 0.82F; // paper sec. 3.1

            const Real3 dragForce = -0.5F * fluidDensity * dragCoefficient *
                                    effectiveArea * relativeSpeed *
                                    relativeVelocity * forceScale;
            totalForce += dragForce;

            // lift force
            const Real3 crossProduct = linalg::cross(normal, relativeVelocity);
            const Real crossLength = linalg::length(crossProduct);

            if (crossLength > 1e-6F) {
                constexpr Real liftCoefficient = 0.075F; // paper sec. 3.1
                const Real3 liftDirection = crossProduct / crossLength;
                const Real3 liftForce =
                  -0.5F * fluidDensity * liftCoefficient * effectiveArea *
                  relativeSpeed * linalg::cross(relativeVelocity, liftDirection) * forceScale;
                totalForce += liftForce;
            }

            //                const Real pressureCoefficient = 0.5F;
            //                const Real3 pressureForce =
            //                  -pressureCoefficient * fluidDensity *
            //                  relativeSpeed * relativeSpeed * normalDotFlow *
            //                  normal;
            //                totalForce += pressureForce;
        }
    }

    return totalForce;
}

// in the 2D axis (considers height to be 0)
Real3 WaterSimulation::getFluidVelocityAtPosition(const Real3& worldPos) const
{
    const Real gridX = (worldPos.x - bottomLeftCornerWorldPos_xz.x) / cellSize;
    const Real gridZ = (worldPos.z - bottomLeftCornerWorldPos_xz.y) / cellSize;

    const Real u = interpolate<numRows, numCols + 1>(
      velocityGrid.getUVelocities(), Real2{gridX + 0.5F, gridZ});

    const Real w = interpolate<numRows + 1, numCols>(
      velocityGrid.getWVelocities(), Real2{gridX, gridZ + 0.5F});

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
                if (!isPositionInWater(static_cast<Real3>(subTriangle.centroid),
                                       heights))
                {
                    continue;
                }

                Real3 triangleVelocity = static_cast<Real3>(
                  rigidBody.getPointVelocity(subTriangle.centroid));

                Real3 fluidVelocity = getFluidVelocityAtPosition(
                  static_cast<Real3>(subTriangle.centroid));

                Real3 relativeVelocity = triangleVelocity - fluidVelocity;

                Real3 fluidForce = computeFluidForceOnTriangle(
                  subTriangle, triangleVelocity, heights);

                rigidBody.applyForce(
                  static_cast<linalg::aliases::float3>(fluidForce),
                  static_cast<linalg::aliases::float3>(subTriangle.centroid));

                rigidBody.dampenAngularMomentum();

                updateFluidWithTriangle(
                  static_cast<Real>(subTriangle.area),
                  static_cast<Real3>(subTriangle.centroid), triangleVelocity,
                  relativeVelocity, static_cast<Real3>(subTriangle.normal),
                  heights);
            }
        }
    }
}
