#pragma once

#include "physics/AABB.hpp"

#include <set>
#include <unordered_map>
#include <vector>

struct SpatialGrid
{
    constexpr static float cellSize =
      2.0F; // Adjust based on typical object size
    std::unordered_map<int64_t, std::vector<size_t>> grid;

    // Simple hash function for 3D grid coordinates
    // TODO: does this work?????
    [[nodiscard]] static constexpr int64_t hashCell(int x, int y, int z)
    {
        return ((int64_t)x * 73856093) ^ ((int64_t)y * 19349663) ^
               ((int64_t)z * 83492791);
    }

    void clear()
    {
        grid.clear();
    }

    void insert(size_t bodyIndex, const AABB& aabb)
    {
        int minX = static_cast<int>(std::floor(aabb.min.x / cellSize));
        int maxX = static_cast<int>(std::floor(aabb.max.x / cellSize));
        int minY = static_cast<int>(std::floor(aabb.min.y / cellSize));
        int maxY = static_cast<int>(std::floor(aabb.max.y / cellSize));
        int minZ = static_cast<int>(std::floor(aabb.min.z / cellSize));
        int maxZ = static_cast<int>(std::floor(aabb.max.z / cellSize));

        for (int x = minX; x <= maxX; ++x) {
            for (int y = minY; y <= maxY; ++y) {
                for (int z = minZ; z <= maxZ; ++z) {
                    int64_t hash = hashCell(x, y, z);
                    grid[hash].push_back(bodyIndex);
                }
            }
        }
    }

    [[nodiscard]] std::vector<std::pair<size_t, size_t>>
    getPotentialCollisions() const
    {
        std::vector<std::pair<size_t, size_t>> pairs;
        std::set<std::pair<size_t, size_t>> uniquePairs;

        for (const auto& cell : grid) {
            const auto& bodies = cell.second;
            for (size_t i = 0; i < bodies.size(); ++i) {
                for (size_t j = i + 1; j < bodies.size(); ++j) {
                    size_t a = std::min(bodies[i], bodies[j]);
                    size_t b = std::max(bodies[i], bodies[j]);
                    uniquePairs.insert({a, b});
                }
            }
        }

        pairs.reserve(uniquePairs.size());
        for (const auto& pair : uniquePairs) {
            pairs.push_back(pair);
        }

        return pairs;
    }
};
