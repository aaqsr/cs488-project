#pragma once

#include "frontend/shader.hpp"
#include "sim/waterSimulation.hpp"

#include <cstdint>
#include <linalg.h>

class Shader;

// Why is this different from frontend/Mesh.hpp?
// Because the the general Mesh's vertices are immutable by design and these are
// not. Should we be using inheritance/composition then with some Base Mesh to
// share common bits? Probably but I really don't care right now...
//
// Update: so it's clear that I really should mix this with frontend/Mesh
// somehow, because I need to add Material and Model functionality.
class WaterMesh
{
    constexpr static size_t numRows = WaterSimulation::numRows;
    constexpr static size_t numCols = WaterSimulation::numCols;
    constexpr static float cellSize = WaterSimulation::cellSize;
    constexpr static linalg::aliases::float2 bottomLeftCornerWorldPos_xz =
      WaterSimulation::bottomLeftCornerWorldPos_xz;

    uint32_t VAO{0};
    uint32_t positionVBO{0};
    uint32_t heightVBO{0};
    uint32_t normalVBO{0};
    uint32_t indexBuffer{0};

    std::array<linalg::aliases::float2,
               numRows * numCols>
      staticPositions; // x, z positions

    std::array<uint32_t, (numRows - 1) * (numCols - 1) * 4> indices;

    std::array<linalg::aliases::float3, numRows * numCols> normals;

    void computeNormals(const std::array<float, numRows * numCols>& heights);

  public:
    explicit WaterMesh();

    WaterMesh(const WaterMesh&) = delete;
    WaterMesh(WaterMesh&&) = delete;
    WaterMesh& operator=(const WaterMesh&) = delete;
    WaterMesh& operator=(WaterMesh&&) = delete;

    ~WaterMesh();

    // Warning: Update does not draw!
    void updateMesh(const std::array<float, numRows * numCols>& heights);

    void draw(Shader::BindObject& shader,
              const linalg::aliases::float3& cameraPos) const;
};
