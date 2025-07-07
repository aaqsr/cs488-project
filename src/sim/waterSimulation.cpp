#include "sim/waterSimulation.hpp"

WaterSimulation::WaterSimulation() : grid{}, mesh{grid.getWaterHeights()}
{
}

void WaterSimulation::update(double deltaTime)
{
    static float phase = 0.0F;
    // speed is 2.0F
    phase += 2.0F * deltaTime;
    for (size_t i = 0; i < numRows; ++i) {
        for (size_t j = 0; j < numCols; ++j) {
            float x = (static_cast<float>(i) - (numRows * 0.5F) + 0.5F) * 0.05F;
            float y = (static_cast<float>(j) - (numCols * 0.5F) + 0.5F) * 0.05F;
            grid.getWaterHeight(i, j) = fn(x, y, phase);
        }
    }
    mesh.updateMesh(grid.getWaterHeights());
}

void WaterSimulation::draw(Shader::BindObject& shader)
{
    mesh.draw(shader);
}

