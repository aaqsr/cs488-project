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

    const std::array<float, numRows*(numCols + 1)>& getUVelocities() const
    {
        return u_velocity;
    }

    const std::array<float, (numRows + 1) * numCols>& getWVelocities() const
    {
        return w_velocity;
    }

    [[nodiscard]]
    float getWaterHeight(size_t i, size_t j) const
    {
        return waterHeight[(i * numCols) + j];
    }

    void setWaterHeight(size_t i, size_t j, float val)
    {
        // water height always clamped to >= 0
        // TODO: any instance where we could accidentally read a value < 0?
        waterHeight[(i * numCols) + j] = std::max(val, 0.0F);
    }

    void addToWaterHeight(size_t i, size_t j, float val)
    {
        // water height always clamped to >= 0
        // TODO: any instance where we could accidentally read a value < 0?
        const float result = getWaterHeight(i, j) + val;
        setWaterHeight(i, j, result);
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
    float getEta(size_t i, size_t j) const
    {
        // return getWaterHeight(i, j) + getTerrainHeight(i, j);
        return getWaterHeight(i, j);
    }

    [[nodiscard]]
    float getVelocity_u_i_plus_half_j(size_t i, size_t j) const
    {
        return u_velocity[(i * (numCols + 1)) + j];
    }
    // maxSpeedClamp used to clamp values. See section 2.1.5.
    void setVelocity_u_i_plus_half_j(size_t i, size_t j, float val,
                                     float maxSpeedClamp)
    {
        // TODO: technically this is less than or equal to, and the paper
        // suggests just less than. But ah well...
        u_velocity[(i * (numCols + 1)) + j] = std::min(val, maxSpeedClamp);
    }
    void addToVelocity_u_i_plus_half_j(size_t i, size_t j, float val,
                                       float maxSpeedClamp)
    {
        const float result = getVelocity_u_i_plus_half_j(i, j) + val;
        setVelocity_u_i_plus_half_j(i, j, result, maxSpeedClamp);
    }

    [[nodiscard]]
    float getVelocity_w_i_j_plus_half(size_t i, size_t j) const
    {
        return w_velocity[(i * numCols) + j];
    }
    void setVelocity_w_i_j_plus_half(size_t i, size_t j, float val,
                                     float maxSpeedClamp)
    {
        w_velocity[(i * numCols) + j] = std::min(val, maxSpeedClamp);
    }
    void addToVelocity_w_i_j_plus_half(size_t i, size_t j, float val,
                                       float maxSpeedClamp)
    {
        const float result = getVelocity_w_i_j_plus_half(i, j) + val;
        setVelocity_w_i_j_plus_half(i, j, result, maxSpeedClamp);
    }
};
