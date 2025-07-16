#include "sim/waterSimulation.hpp"
#include "linalg.h"
#include "sim/waterHeightGrid.hpp"
#include "util/error.hpp"
#include <algorithm>
#include <array>
#include <iterator>

namespace
{

template <size_t rows, size_t cols>
[[nodiscard]]
float interpolate(const std::array<float, rows * cols>& field,
                  linalg::aliases::float2 pos)
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

void setInitConditions(
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

} // namespace

WaterSimulation::WaterSimulation() = default;

void WaterSimulation::update()
{
    if (!isPlaying) {
        return;
    }

    if (channel == nullptr) {
        throw IrrecoverableError{
          "Channel is nullptr in WaterSimulation::update()"};
    }

    advectVelocities();

    {
        // message auto sends at end of scope
        auto message = channel->createMessage();
        auto& newHeightGrid = message.getWriteBuffer();
        const auto& prevHeightGrid = message.getPreviousWriteBuffer();

        for (size_t i = 0; i < numRows; ++i) {
            for (size_t j = 0; j < numCols; ++j) {
                const float newHeight =
                  prevHeightGrid.getWaterHeight(i, j) +
                  calcHeightChangeIntegral(i, j, prevHeightGrid);
                newHeightGrid.setWaterHeight(i, j, newHeight, maxDepth);
            }
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
    constexpr float g_over_deltaX = -gravitationalAcceleration / deltaX;

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

    // return nextGrid;
}

void WaterSimulation::togglePlay()
{
    isPlaying = !isPlaying;
}

void WaterSimulation::attachSenderChannel(
  Sender<HeightGrid<numRows, numCols>>* s)
{
    if (s == nullptr) {
        throw IrrecoverableError{
          "Channel is nullptr in WaterSimulation::attachSenderChannel()"};
    }

    channel = s;

    {
        auto msg = channel->createMessage();
        setInitConditions(msg.getWriteBuffer());
    }
}
