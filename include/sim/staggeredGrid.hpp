#pragma once

#include "util/logger.hpp"

#include <array>

template <int numRows, int numCols>
class StaggeredGrid
{
  private:
    // TODO: use std::array actually probably
    // Stored at centre of cells
    std::array<float, numRows * numCols> waterHeight; // h_{i,j}

    // Constant so not needed for now
    // std::array<float, numRows * numCols> terrainHeight; // H_{i,j}

    // Stored at faces of the cells
    // v = (u, w) \in \R^2 (u in x-dir, w in z-dir)
    std::array<float, numRows*(numCols + 1)>
      u_velocity; // We store u_{i+1/2, j}
    std::array<float, (numRows + 1) * numCols>
      w_velocity; // We store w_{i, j+1/2}

    // TODO: what to do of grid spacing delta x, time spacing delta t?

    constexpr static auto fn = [](float x, float y, float phase = 0.0F) {
        return std::cos(std::sqrtf((x * x) + (y * y)) + phase) * 1.0F;
    };

  public:
    StaggeredGrid()
    {
        // TODO: temporary plz
        for (int i = 0; i < numRows; ++i) {
            for (int j = 0; j < numCols; ++j) {
                float x = (i - (numRows * 0.5F)) * 0.1F;
                float y = (j - (numCols * 0.5F)) * 0.1F;
                waterHeight[(i * numCols) + j] = fn(x, y);
            }
        }
        // Logger::GetInstance().log("Constructing grid with init values of",
        //                           waterHeight, numRows, numCols);
    }

    void updateGrid(float deltaTime)
    {
        static float phase = 0.0F;
        // speed is 2.0F
        phase += 2.0F * deltaTime;
        for (int i = 0; i < numRows; ++i) {
            for (int j = 0; j < numCols; ++j) {
                float x = (i - (numRows * 0.5F)) * 0.1F;
                float y = (j - (numCols * 0.5F)) * 0.1F;
                waterHeight[(i * numCols) + j] = fn(x, y, phase);
            }
        }
    }

    float getWaterHeight(int i, int j)
    {
        return waterHeight[(i * numCols) + j];
    }

    // float getTerrainHeight(int i, int j)
    // {
    //     return terrainHeight[(i * numCols) + j];
    // }

    [[nodiscard]]
    const std::array<float, numRows * numCols>& getWaterHeights() const
    {
        return waterHeight;
    }

    float getEta(int i, int j)
    {
        // return getWaterHeight(i, j) + getTerrainHeight(i, j);
        return getWaterHeight(i, j);
    }

    float getVelocity_u_i_plus_half_j(int i, int j)
    {
        return u_velocity[(i * (numCols + 1)) + j];
    }

    float getVelocity_w_i_j_plus_half(int i, int j)
    {
        return w_velocity[(i * numCols) + j];
    }
};
