#pragma once

#include "frontend/shader.hpp"
#include "physics/AABB.hpp"
#include "util/logger.hpp"
#include <GL/glew.h>
#include <filesystem>
#include <linalg.h>
#include <vector>

class AABBVisualiser
{
  private:
    Shader shader;

    GLuint VAO = 0;
    GLuint VBO = 0;
    GLuint instanceVBO = 0; // instance buffer for model matrices
    GLuint EBO = 0;

    // unit cube centered at origin
    static constexpr float cubeVertices[] = {
      // Front face
      -0.5F, -0.5F, 0.5F, 0.5F, -0.5F, 0.5F, 0.5F, 0.5F, 0.5F, -0.5F, 0.5F,
      0.5F,

      // Back face
      -0.5F, -0.5F, -0.5F, 0.5F, -0.5F, -0.5F, -0.5F, 0.5F, -0.5F, 0.5F, 0.5F,
      -0.5F,

      // Left face
      -0.5F, -0.5F, -0.5F, -0.5F, -0.5F, 0.5F, -0.5F, 0.5F, 0.5F, -0.5F, 0.5F,
      -0.5F,

      // Right face
      0.5F, -0.5F, 0.5F, 0.5F, -0.5F, -0.5F, 0.5F, 0.5F, -0.5F, 0.5F, 0.5F,
      0.5F,

      // Top face
      -0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F, -0.5F, -0.5F, 0.5F,
      -0.5F,

      // Bottom face
      -0.5F, -0.5F, -0.5F, 0.5F, -0.5F, -0.5F, 0.5F, -0.5F, 0.5F, -0.5F, -0.5F,
      0.5F};

    // wireframe cube indices
    static constexpr unsigned int cubeIndices[] = {
      // Front face
      0, 1, 2, 2, 3, 0,
      // Back face
      5, 4, 6, 6, 7, 5,
      // Left face
      8, 9, 10, 10, 11, 8,
      // Right face
      12, 13, 14, 14, 15, 12,
      // Top face
      16, 17, 18, 18, 19, 16,
      // Bottom face
      20, 21, 22, 22, 23, 20};

    // instance data storage
    std::vector<linalg::aliases::float4x4> modelMatrices;
    size_t maxInstances = 50; // initial capacity

    void setupBuffers()
    {
        glGenVertexArrays(1, &VAO);
        glBindVertexArray(VAO);

        glGenBuffers(1, &VBO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), cubeVertices,
                     GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                              (void*)0);
        glEnableVertexAttribArray(0);

        glGenBuffers(1, &EBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(cubeIndices), cubeIndices,
                     GL_STATIC_DRAW);

        glGenBuffers(1, &instanceVBO);
        glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);

        modelMatrices.reserve(maxInstances);
        glBufferData(GL_ARRAY_BUFFER,
                     maxInstances * sizeof(linalg::aliases::float4x4), nullptr,
                     GL_DYNAMIC_DRAW);

        // setup instance attributes (model matrix columns)
        // model matrix column 0 (location 1)
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE,
                              sizeof(linalg::aliases::float4x4),
                              (void*)nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribDivisor(1, 1);

        // model matrix column 1 (location 2)
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE,
                              sizeof(linalg::aliases::float4x4),
                              (void*)(sizeof(linalg::aliases::float4)));
        glEnableVertexAttribArray(2);
        glVertexAttribDivisor(2, 1);

        // model matrix column 2 (location 3)
        glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE,
                              sizeof(linalg::aliases::float4x4),
                              (void*)(2 * sizeof(linalg::aliases::float4)));
        glEnableVertexAttribArray(3);
        glVertexAttribDivisor(3, 1);

        // model matrix column 3 (location 4)
        glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE,
                              sizeof(linalg::aliases::float4x4),
                              (void*)(3 * sizeof(linalg::aliases::float4)));
        glEnableVertexAttribArray(4);
        glVertexAttribDivisor(4, 1);

        // clean-up - unbind buffers
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    [[nodiscard]] static linalg::aliases::float4x4
    createModelMatrix(const AABB& aabb)
    {
        linalg::aliases::float3 center = (aabb.max + aabb.min) * 0.5F;
        linalg::aliases::float3 size = aabb.max - aabb.min;

        // create transformation matrix: T * S
        // translation to centre, then scale to size
        return linalg::mul(linalg::translation_matrix(center),
                           linalg::scaling_matrix(size));
    }

  public:
    AABBVisualiser()
      : shader(
          std::filesystem::path{"shaders/vertex/debugInstancedShader.glsl"},
          std::filesystem::path{"shaders/fragment/uniformColour.glsl"},
          std::vector<std::string>{"view", "projection", "colour"})
    {
        setupBuffers();
    }

    ~AABBVisualiser()
    {
        if (VAO != 0) {
            glDeleteVertexArrays(1, &VAO);
        }
        if (VBO != 0) {
            glDeleteBuffers(1, &VBO);
        }
        if (instanceVBO != 0) {
            glDeleteBuffers(1, &instanceVBO);
        }
        if (EBO != 0) {
            glDeleteBuffers(1, &EBO);
        }
    }

    AABBVisualiser(const AABBVisualiser&) = delete;
    AABBVisualiser& operator=(const AABBVisualiser&) = delete;
    AABBVisualiser(AABBVisualiser&&) = delete;
    AABBVisualiser& operator=(AABBVisualiser&&) = delete;

    void draw(const std::vector<AABB>& aabbs,
              const linalg::aliases::float4x4& viewMatrix,
              const linalg::aliases::float4x4& projMatrix,
              const linalg::aliases::float4& colour = {1.0F, 0.0F, 0.0F, 1.0F})
    {
        if (aabbs.empty()) {
            return;
        }

        // resize buffer if needed
        if (aabbs.size() > maxInstances) {
            setMaxInstances(aabbs.size() * 2); // Double the capacity
        }

        // clear and populate model matrices
        modelMatrices.clear();
        modelMatrices.reserve(aabbs.size());

        for (const AABB& aabb : aabbs) {
            modelMatrices.push_back(createModelMatrix(aabb));
        }

        // update instance buffer with new data
        glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        modelMatrices.size() *
                          sizeof(linalg::aliases::float4x4),
                        modelMatrices.data());
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        // enable wireframe mode for AABB visualization
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        {
            auto boundShader = shader.bind();
            boundShader.setUniform("view", viewMatrix);
            boundShader.setUniform("projection", projMatrix);
            boundShader.setUniform("colour", colour);

            // Bind VAO and draw instances
            glBindVertexArray(VAO);
            glDrawElementsInstanced(
              GL_TRIANGLES, sizeof(cubeIndices) / sizeof(unsigned int),
              GL_UNSIGNED_INT, nullptr, static_cast<GLsizei>(aabbs.size()));
            glBindVertexArray(0);
        }

        GLenum error = glGetError();
        if (error != GL_NO_ERROR) {
            Logger::GetInstance().log("AABB Visualiser Draw Error Code " +
                                      std::to_string(error));
        }

        // Restore fill mode
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    void setMaxInstances(size_t newMaxInstances)
    {
        if (newMaxInstances <= maxInstances) {
            return;
        }

        maxInstances = newMaxInstances;
        modelMatrices.reserve(maxInstances);

        // Reallocate instance buffer
        glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
        glBufferData(GL_ARRAY_BUFFER,
                     maxInstances * sizeof(linalg::aliases::float4x4), nullptr,
                     GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
};
