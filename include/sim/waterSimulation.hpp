#pragma once

#include "sim/staggeredGrid.hpp"
#include "sim/waterMesh.hpp"

class WaterSimulation
{
  public:
    // TODO: come up with actual numbers
    static constexpr size_t numRows = 200;
    static constexpr size_t numCols = 100;
    constexpr static float cellSize = 0.05F;

  private:
    StaggeredGrid<numRows, numCols> grid;
    WaterMesh<numRows, numCols, cellSize> mesh;

    constexpr static auto fn = [](float x, float y, float phase = 0.0F) {
        return std::cos(std::sqrtf((x * x) + (y * y)) + phase) * 1.0F;
    };

  public:
    WaterSimulation();

    void update(double deltaTime);
    void draw(Shader::BindObject& shader);
};
