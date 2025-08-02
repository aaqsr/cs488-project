#pragma once

#include "physics/AABB.hpp"

#include <set>
#include <unordered_map>
#include <vector>

/**
 * @struct SpatialGrid
 * @brief A uniform spatial grid for accelerating broad-phase collision
 * detection.
 * @ingroup physics
 *
 * @details This data structure partitions 3D space into a grid of uniform
 * cells. Objects are inserted into every cell their AABB overlaps. To find
 * potential collisions for an object, one only needs to check against the other
 * objects in the cells it occupies.
 *
 * @section Technicality
 * This is a form of "spatial hashing". Instead of allocating a 3D array for the
 * grid (which could be enormous and sparse), a hash map is used to store only
 * the non-empty cells. A hash function converts a 3D cell index `(x, y, z)`
 * into a single 64-bit key.
 *
 * @section Performance
 * - **Complexity:** For a world with N objects, a brute-force check is O(N^2).
 * With a spatial grid, the average-case complexity for finding all collision
 * pairs is closer to O(N*k), where k is the average number of objects per cell.
 * This is a significant improvement in typical scenarios.
 * - **`cellSize`:** The choice of `cellSize` is a critical performance tuning
 * parameter.
 * - If too small, objects will span many cells, increasing insertion cost.
 * - If too large, cells will contain too many objects, and the performance will
 * degrade towards O(N^2). A good heuristic is to set it to be around the size
 * of the average object in the simulation.
 *
 * @section Caveats
 * The hash function uses large prime numbers to reduce the probability of
 * collisions (two different `(x,y,z)` coordinates mapping to the same hash
 * key). While effective, it is not guaranteed to be perfect. A more robust
 * implementation might use a more sophisticated spatial hash function.
 */
struct SpatialGrid
{
    /**
     * @brief The side length of each cubic cell in the grid. This is a crucial
     * tuning parameter.
     */
    constexpr static float cellSize = 2.0F;
    /**
     * @brief The core data structure. Maps a cell's hash key to a vector of
     * indices of the rigid bodies that overlap that cell.
     */
    std::unordered_map<int64_t, std::vector<size_t>> grid;

    /**
     * @brief A hash function to convert a 3D integer cell coordinate into a
     * single 64-bit integer key.
     * @param x The x-index of the cell.
     * @param y The y-index of the cell.
     * @param z The z-index of the cell.
     * @return A 64-bit hash value.
     */
    [[nodiscard]] static constexpr int64_t hashCell(int x, int y, int z)
    {
        return ((int64_t)x * 73856093) ^ ((int64_t)y * 19349663) ^
               ((int64_t)z * 83492791);
    }

    /**
     * @brief Clears the grid, preparing it for the next simulation frame.
     */
    void clear()
    {
        grid.clear();
    }

    /**
     * @brief Inserts a rigid body's AABB into the grid.
     * @details The method determines all grid cells the AABB overlaps and adds
     * the body's index to each of those cells.
     * @param bodyIndex The index of the body in the main physics engine array.
     * @param aabb The world-space AABB of the body.
     */
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

    /**
     * @brief Generates a list of all unique pairs of bodies that might be
     * colliding.
     * @details It iterates through each cell in the grid. For any cell
     * containing more than one body, all unique pairs of bodies within that
     * cell are added to the list of potential collisions. A `std::set` is used
     * to ensure that no pair is reported more than once.
     * @return A vector of pairs, where each pair contains the indices of two
     * potentially colliding bodies.
     */
    [[nodiscard]] std::vector<std::pair<size_t, size_t>>
    getPotentialCollisions() const
    {
        std::vector<std::pair<size_t, size_t>> pairs;
        std::set<std::pair<size_t, size_t>> uniquePairs;

        for (const auto& cell : grid) {
            const auto& bodies = cell.second;
            if (bodies.size() > 1) {
                for (size_t i = 0; i < bodies.size(); ++i) {
                    for (size_t j = i + 1; j < bodies.size(); ++j) {
                        size_t a = std::min(bodies[i], bodies[j]);
                        size_t b = std::max(bodies[i], bodies[j]);
                        uniquePairs.insert({a, b});
                    }
                }
            }
        }
        pairs.assign(uniquePairs.begin(), uniquePairs.end());
        return pairs;
    }
};
