#include "sim/waterSimulation.hpp"
#include <array>

namespace
{

template <size_t rows, size_t cols>
[[nodiscard]]
float interpolate(const std::array<float, rows * cols>& field,
                  linalg::aliases::float2 pos)
{
    // clamp coords to be within the valid grid area. subtract 1 from
    // dimensions because we need 4 points for interpolation.
    pos.x = std::max(0.0F, std::min(pos.x, static_cast<float>(cols - 1.0001F)));
    // why subtracting 1.0001??
    pos.y = std::max(0.0F, std::min(pos.y, static_cast<float>(rows - 1.0001F)));

    const int i0 = static_cast<int>(pos.x);
    const int j0 = static_cast<int>(pos.y);
    const int i1 = i0 + 1;
    const int j1 = j0 + 1;

    const float tx = pos.x - static_cast<float>(i0);
    const float ty = pos.y - static_cast<float>(j0);

    const float v00 = field[(j0 * cols) + i0];
    const float v10 = field[(j0 * cols) + i1];
    const float v01 = field[(j1 * cols) + i0];
    const float v11 = field[(j1 * cols) + i1];

    // Linear interpolation in x
    const float lerpX1 = v00 + ((v10 - v00) * tx);
    const float lerpX2 = v01 + ((v11 - v01) * tx);

    // Linear interpolation in y
    return lerpX1 + ((lerpX2 - lerpX1) * ty);
}

} // namespace

WaterSimulation::WaterSimulation() : grid{}, mesh{grid.getWaterHeights()}
{
    // Initial Conditions
    constexpr static auto fn = [](float x, float y, float phase = 0.0F) {
        return std::cos(std::sqrtf((x * x) + (y * y)) + phase) * 1.0F;
        // return std::cos(x + phase) * 1.0F;
    };

    const float phase = 0.0F;

    // speed is 2.0F
    for (size_t i = 0; i < WaterSimulation::numRows; ++i) {
        for (size_t j = 0; j < WaterSimulation::numCols; ++j) {
            float x = (static_cast<float>(i) -
                       (WaterSimulation::numRows * 0.5F) + 0.5F) *
                      cellSize;
            float y = (static_cast<float>(j) -
                       (WaterSimulation::numCols * 0.5F) + 0.5F) *
                      cellSize;
            grid.setWaterHeight(i, j, (0.5F * fn(x, y, phase)) + 1.0F);
        }
    }
}

void WaterSimulation::update()
{
    if (!isPlaying) {
        return;
    }

    const float deltaTime = deltaT;

    advectVelocities(grid, deltaTime);

    for (size_t i = 0; i < numRows; ++i) {
        for (size_t j = 0; j < numCols; ++j) {
            grid.addToWaterHeight(i, j,
                                  calcHeightChangeIntegral(i, j, deltaTime));
        }
    }

    for (size_t i = 0; i < numRows; ++i) {
        for (size_t j = 0; j < numCols; ++j) {
            const float maxSpeedClamp = getMaxSpeedClamp(deltaTime);
            const linalg::aliases::float2 velChange =
              calcVelocityChangeIntegration(i, j, deltaTime);
            grid.addToVelocity_u_i_plus_half_j(i, j, velChange.x,
                                               maxSpeedClamp);
            grid.addToVelocity_w_i_j_plus_half(i, j, velChange.y,
                                               maxSpeedClamp);
        }
    }

    // static int numTimes = 0;
    // if (numTimes < 5) {
    //   ++numTimes;
    //
    // }
}

void WaterSimulation::draw(Shader::BindObject& shader)
{
    mesh.updateMesh(grid.getWaterHeights());
    mesh.draw(shader);
}

float WaterSimulation::calcHeightChangeIntegral(size_t i, size_t j,
                                                float deltaTime) const
{
    // Both evaluated in the upwind direction
    const auto hBar_i_plus_half_j = [](const decltype(grid)& grid, size_t i,
                                       size_t j) -> float {
        return (grid.getVelocity_u_i_plus_half_j(i, j) <= 0)
                 ? grid.getWaterHeight(i + 1, j)
                 : grid.getWaterHeight(i, j);
    };
    const auto hBar_i_j_plus_half = [](const decltype(grid)& grid, size_t i,
                                       size_t j) -> float {
        return (grid.getVelocity_w_i_j_plus_half(i, j) <= 0)
                 ? grid.getWaterHeight(i, j + 1)
                 : grid.getWaterHeight(i, j);
    };

    // TODO: Stability improvement in section 2.1.5 final paragraph
    // constexpr float beta = 2.0F;
    // const float h_avgmax = (beta * deltaX) / (gravitationalAcceleration *
    // deltaTime);

    // TODO: Within bounds?
    // Also TODO: size_t overflow if less than zero and we are subtracting
    // one here
    const float uDirectionNumerator =
      (hBar_i_plus_half_j(grid, i, j) *
       grid.getVelocity_u_i_plus_half_j(i, j)) -
      (hBar_i_plus_half_j(grid, i - 1, j) *
       grid.getVelocity_u_i_plus_half_j(i - 1, j));

    const float wDirectionNumerator =
      (hBar_i_j_plus_half(grid, i, j) *
       grid.getVelocity_w_i_j_plus_half(i, j)) -
      (hBar_i_j_plus_half(grid, i, j - 1) *
       grid.getVelocity_w_i_j_plus_half(i, j - 1));

    constexpr float invDeltaX = 1.0F / deltaX;

    const float delH_delT =
      -(uDirectionNumerator + wDirectionNumerator) * invDeltaX;

    return delH_delT * deltaTime;
}

linalg::aliases::float2 WaterSimulation::calcVelocityChangeIntegration(
  size_t i, size_t j, float deltaTime,
  const linalg::aliases::float3& accelExt) const
{
    const float eta_ij = grid.getEta(i, j);
    constexpr float g_over_deltaX = -gravitationalAcceleration * deltaX;

    const float delta_u_i_plus_half_j =
      (g_over_deltaX * (grid.getEta(i + 1, j) - eta_ij) + accelExt.x) *
      deltaTime;
    const float delta_w_i_j_plus_half =
      (g_over_deltaX * (grid.getEta(i, j + 1) - eta_ij) + accelExt.z) *
      deltaTime;

    return {delta_u_i_plus_half_j, delta_w_i_j_plus_half};
}

// std::unique_ptr<
//   StaggeredGrid<WaterSimulation::numRows, WaterSimulation::numCols>>
void WaterSimulation::advectVelocities(
  const StaggeredGrid<numRows, numCols>& currentGrid, float deltaTime)
{
    // auto nextGrid = std::make_unique<StaggeredGrid<numRows, numCols>>();
    auto& nextGrid = grid;

    // Advect u
    for (size_t j = 0; j < numRows; ++j) {
        for (size_t i = 0; i < numCols + 1; ++i) {
            // pos of u-velocity component
            linalg::aliases::float2 pos = {(static_cast<float>(i) - 0.5F) *
                                             deltaX,
                                           static_cast<float>(j) * deltaX};

            // interpolate the w-velocity at this position to get a full
            // velocity vector. Grid coordinates for w-field interpolation
            linalg::aliases::float2 w_pos_grid = {pos.x / deltaX,
                                                  (pos.y / deltaX) - 0.5F};
            const float w_interp = interpolate<numRows + 1, numCols>(
              currentGrid.getWVelocities(), w_pos_grid);

            linalg::aliases::float2 vel = {
              currentGrid.getVelocity_u_i_plus_half_j(j, i), w_interp};

            // trace back in time
            linalg::aliases::float2 departure_pos = pos - vel * deltaTime;

            // sample old u-velocity field at departure point.
            // convert departure point in to a u-field grid coords
            linalg::aliases::float2 u_pos_grid = {
              (departure_pos.x / deltaX) + 0.5F, departure_pos.y / deltaX};
            const float new_u = interpolate<numRows, numCols + 1>(
              currentGrid.getUVelocities(), u_pos_grid);

            nextGrid.setVelocity_u_i_plus_half_j(j, i, new_u,
                                                 100.0F /*max speed clamp*/);
        }
    }

    // Advect w
    for (size_t j = 0; j < numRows + 1; ++j) {
        for (size_t i = 0; i < numCols; ++i) {
            linalg::aliases::float2 pos = {static_cast<float>(i) * deltaX,
                                           (static_cast<float>(j) - 0.5F) *
                                             deltaX};

            linalg::aliases::float2 u_pos_grid = {(pos.x / deltaX) - 0.5F,
                                                  pos.y / deltaX};
            const float u_interp = interpolate<numRows, numCols + 1>(
              currentGrid.getUVelocities(), u_pos_grid);

            linalg::aliases::float2 vel = {
              u_interp, currentGrid.getVelocity_w_i_j_plus_half(j, i)};

            linalg::aliases::float2 departure_pos = pos - vel * deltaTime;

            linalg::aliases::float2 w_pos_grid = {
              departure_pos.x / deltaX, (departure_pos.y / deltaX) + 0.5F};
            const float new_w = interpolate<numRows + 1, numCols>(
              currentGrid.getWVelocities(), w_pos_grid);

            nextGrid.setVelocity_w_i_j_plus_half(j, i, new_w,
                                                 100.0F /*max speed clamp*/);
        }
    }

    // return nextGrid;
}

float WaterSimulation::getMaxSpeedClamp(float deltaTime)
{
    constexpr float alpha = 0.5F;
    return alpha * cellSize / deltaTime;
}
void WaterSimulation::togglePlay()
{
    isPlaying = !isPlaying;
}
