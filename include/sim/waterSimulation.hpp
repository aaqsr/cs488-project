#pragma once

#include "sim/staggeredGrid.hpp"

class WaterSimulation
{
  public:
    // TODO: come up with actual numbers
    static constexpr int numRows = 15;
    static constexpr int numCols = 10;

  private:
    StaggeredGrid<numRows, numCols> grid;
};
