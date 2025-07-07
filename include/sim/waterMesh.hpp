#pragma once

#include "frontend/shader.hpp"
#include "util/logger.hpp"

#include <GL/glew.h>
#include <linalg.h>

#include <cstdint>
#include <string>

// Why is this different from frontend/Mesh.hpp?
// Because the the general Mesh's vertices are immutable by design and these are
// not. Should we be using inheritance/composition then with some Base Mesh to
// share common bits? Probably but I really don't care right now...
//
// Update: so it's clear that I really should mix this with frontend/Mesh
// somehow, because I need to add Material and Model functionality.
template <size_t numRows, size_t numCols, float cellSize>
class WaterMesh
{
    uint32_t VAO{0};
    uint32_t positionVBO{0};
    uint32_t heightVBO{0};
    uint32_t indexBuffer{0};

    std::array<linalg::aliases::float2,
               numRows * numCols>
      staticPositions; // x, z positions

    std::array<uint32_t, (numRows - 1) * (numCols - 1) * 4> indices;

  public:
    WaterMesh(const std::array<float, numRows * numCols>& initHeights)
    {
        // Static positions
        for (int j = 0; j < numRows; ++j) {
            for (int i = 0; i < numCols; ++i) {
                // Store x,z positions (these never change)
                staticPositions[(j * numCols) + i] = {
                  static_cast<float>(i) * cellSize,
                  static_cast<float>(j) * cellSize};
            }
        }
        // Logger::GetInstance().log("Static positions,", staticPositions,
        // numRows,
        //                           numCols);

        int indexPos = 0;
        for (int j = 0; j < numRows - 1; ++j) {
            for (int i = 0; i < numCols - 1; ++i) {
                // vertices in counter-clockwise order
                int topLeft = (j * numCols) + i;
                int topRight = (j * numCols) + (i + 1);
                int bottomLeft = ((j + 1) * numCols) + i;
                int bottomRight = ((j + 1) * numCols) + (i + 1);

                indices[indexPos++] = (bottomLeft);
                indices[indexPos++] = (bottomRight);
                indices[indexPos++] = (topRight);
                indices[indexPos++] = (topLeft);
            }
        }
        // Logger::GetInstance().log("Indices,", indices);

        // OpenGL buffer setup
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &positionVBO);
        glGenBuffers(1, &heightVBO);
        glGenBuffers(1, &indexBuffer);

        glBindVertexArray(VAO);

        // setup static position buffer (x,z coordinates)
        glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
        glBufferData(GL_ARRAY_BUFFER,
                     staticPositions.size() * 2 * sizeof(float),
                     staticPositions.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void*)nullptr);

        // setup dynamic height buffer (y coordinates of grid)
        glBindBuffer(GL_ARRAY_BUFFER, heightVBO);
        glBufferData(GL_ARRAY_BUFFER, (numRows * numCols) * sizeof(float),
                     initHeights.data(), GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 0, (void*)nullptr);

        // setup index buffer
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     indices.size() * sizeof(unsigned int), indices.data(),
                     GL_STATIC_DRAW);

        glBindVertexArray(0);

        // for (int i = 0; i < 4; i++) {
        //     int idx = indices[i];
        //     Logger::GetInstance().log(
        //       "indices[" + std::to_string(i) + "] = " + std::to_string(idx) +
        //       " -> pos (" + std::to_string(staticPositions[idx].x) + ", 0," +
        //       std::to_string(staticPositions[idx].y) + ")");
        // }
    }

    WaterMesh(const WaterMesh&) = delete;
    WaterMesh(WaterMesh&&) = delete;
    WaterMesh& operator=(const WaterMesh&) = delete;
    WaterMesh& operator=(WaterMesh&&) = delete;

    ~WaterMesh()
    {
        if (VAO != 0) {
            glDeleteVertexArrays(1, &VAO);
            glDeleteBuffers(1, &positionVBO);
            glDeleteBuffers(1, &heightVBO);
            glDeleteBuffers(1, &indexBuffer);
        }
    }

    // Warning: Update does not draw!
    void updateMesh(const std::array<float, numRows * numCols>& heights)
    {
        // upload only the height data
        glBindBuffer(GL_ARRAY_BUFFER, heightVBO);

        // orphan old buffer so that we don't have to wait for the GPU to be
        // done with it GPU *should* clean up the old buffer for us/
        // TODO: is this implementation correct...
        glBufferData(GL_ARRAY_BUFFER, heights.size() * sizeof(float), nullptr,
                     GL_DYNAMIC_DRAW);
        glBufferSubData(GL_ARRAY_BUFFER, 0, heights.size() * sizeof(float),
                        heights.data());
    }

    void draw(Shader::BindObject& shader) const
    {
        // TODO: integrate this into the Model class and not here??
        shader.setUniform("model", linalg::aliases::float4x4(linalg::identity));

        glBindVertexArray(VAO);

        // transparency??
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // TODO: maybe have an actual material stored on the mesh that is drawn

        // If using patches (needs OpenGL 4.1)
        // glPatchParameteri(GL_PATCH_VERTICES, 4);
        glDrawElements(GL_LINES_ADJACENCY, indices.size(), GL_UNSIGNED_INT, 0);

        // do we have to disable it?
        glDisable(GL_BLEND);
        glBindVertexArray(0);
    }
};
