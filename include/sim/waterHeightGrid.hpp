#pragma once


#include "physics/constants.hpp"

#include <algorithm>
#include <array>
#include <cstddef>

template <size_t numRows, size_t numCols>
class HeightGrid
{
    // Stored at centre of cells
    std::array<float, numRows * numCols> waterHeight; // h_{i,j}

    // Constant so not needed for now
    // std::array<float, numRows * numCols> terrainHeight; // H_{i,j}

  public:
    [[nodiscard]]
    float getWaterHeight(size_t i, size_t j) const
    {
        return waterHeight[(i * numCols) + j];
    }

    void setWaterHeight(size_t i, size_t j, float val)
    {
        // water height always clamped to >= 0
        // TODO: any instance where we could accidentally read a value < 0?
        waterHeight[(i * numCols) + j] =
          std::clamp(val, 0.0F, Physics::WaterSim::maxDepth);
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
};
