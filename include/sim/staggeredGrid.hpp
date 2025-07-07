#pragma once

#include "util/logger.hpp"

#include <array>

template <size_t numRows, size_t numCols>
class StaggeredGrid
{
  private:
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

  public:
    StaggeredGrid() = default;
    StaggeredGrid(const StaggeredGrid&) = delete;
    StaggeredGrid(StaggeredGrid&&) = delete;
    StaggeredGrid& operator=(const StaggeredGrid&) = delete;
    StaggeredGrid& operator=(StaggeredGrid&&) = delete;
    ~StaggeredGrid() = default;

    [[nodiscard]]
    float& getWaterHeight(size_t i, size_t j)
    {
        return waterHeight[(i * numCols) + j];
    }
    [[nodiscard]]
    float getWaterHeight(size_t i, size_t j) const
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

    [[nodiscard]]
    float getEta(int i, int j)
    {
        // return getWaterHeight(i, j) + getTerrainHeight(i, j);
        return getWaterHeight(i, j);
    }

    [[nodiscard]]
    float& getVelocity_u_i_plus_half_j(int i, int j)
    {
        return u_velocity[(i * (numCols + 1)) + j];
    }
    [[nodiscard]]
    float getVelocity_u_i_plus_half_j(int i, int j) const
    {
        return u_velocity[(i * (numCols + 1)) + j];
    }

    [[nodiscard]]
    float& getVelocity_w_i_j_plus_half(int i, int j)
    {
        return w_velocity[(i * numCols) + j];
    }
    [[nodiscard]]
    float getVelocity_w_i_j_plus_half(int i, int j) const
    {
        return w_velocity[(i * numCols) + j];
    }
};
