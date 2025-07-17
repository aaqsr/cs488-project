#pragma once

#include "util/error.hpp"
#include "util/logger.hpp"

#include <array>

template <size_t numRows, size_t numCols>
class StaggeredVelocityGrid
{
  private:
    // Stored at faces of the cells
    // v = (u, w) \in \R^2 (u in x-dir, w in z-dir)
    std::array<float, numRows*(numCols + 1)>
      u_velocity; // We store u_{i+1/2, j}
    std::array<float, (numRows + 1) * numCols>
      w_velocity; // We store w_{i, j+1/2}

    float maxSpeedComponent;

  public:
    explicit StaggeredVelocityGrid(float maxSpeedClamp)
      : maxSpeedComponent{maxSpeedClamp}
    {
        if (maxSpeedClamp < 0) {
            throw IrrecoverableError{"Speed should be unsigned magnitude"};
        }
    }
    StaggeredVelocityGrid(const StaggeredVelocityGrid&) = delete;
    StaggeredVelocityGrid(StaggeredVelocityGrid&&) = default;
    StaggeredVelocityGrid& operator=(const StaggeredVelocityGrid&) = delete;
    StaggeredVelocityGrid& operator=(StaggeredVelocityGrid&&) = delete;
    ~StaggeredVelocityGrid() = default;

    constexpr static size_t m_numRows = numRows;
    constexpr static size_t m_numCols = numCols;

    const std::array<float, numRows*(numCols + 1)>& getUVelocities() const
    {
        return u_velocity;
    }

    const std::array<float, (numRows + 1) * numCols>& getWVelocities() const
    {
        return w_velocity;
    }

    [[nodiscard]]
    float getVelocity_u_i_plus_half_j(size_t i, size_t j) const
    {
        return u_velocity[(i * (numCols + 1)) + j];
    }

    float clampVelocity(float velocity)
    {
        float magnitude = std::abs(velocity);
        float clampedMagnitude = std::min(magnitude, maxSpeedComponent);
        return std::copysign(clampedMagnitude, velocity);
    }

    // maxSpeedClamp used to clamp values. See section 2.1.5.
    void setVelocity_u_i_plus_half_j(size_t i, size_t j, float val)

    {
        // TODO: technically this is less than or equal to, and the paper
        // suggests just less than. But ah well...
        u_velocity[(i * (numCols + 1)) + j] = clampVelocity(val);
    }

    [[nodiscard]]
    float getVelocity_w_i_j_plus_half(size_t i, size_t j) const
    {
        return w_velocity[(i * numCols) + j];
    }
    void setVelocity_w_i_j_plus_half(size_t i, size_t j, float val)
    {
        w_velocity[(i * numCols) + j] = clampVelocity(val);
    }
};
