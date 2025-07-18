#pragma once

#include "frontend/shader.hpp"
#include "linalg.h"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

class Camera;

// At this point, who cares about code reuse...it's faster to reinvent the wheel
// And at the end of the day, results matter...not OOP code quality :(
class Skybox
{
    uint32_t VAO{0}, VBO{0}, EBO{0}, textureId{0};

    std::vector<linalg::aliases::float3> skyboxVertices{
      {-1.0F, -1.0F,  1.0F},
      { 1.0F, -1.0F,  1.0F},
      { 1.0F, -1.0F, -1.0F},
      {-1.0F, -1.0F, -1.0F},
      {-1.0F,  1.0F,  1.0F},
      { 1.0F,  1.0F,  1.0F},
      { 1.0F,  1.0F, -1.0F},
      {-1.0F,  1.0F, -1.0F}
    };

    std::vector<uint32_t> skyboxIndices{// Right
                                        1, 5, 6, 6, 2, 1,
                                        // Left
                                        0, 3, 7, 7, 4, 0,
                                        // Top
                                        4, 7, 6, 6, 5, 4,
                                        // Bottom
                                        0, 1, 2, 2, 3, 0,
                                        // Back
                                        0, 4, 5, 5, 1, 0,
                                        // Front
                                        3, 2, 6, 6, 7, 3};

    const std::filesystem::path textureDirectory =
      "assets/skyboxes/Standard-Cube-Map";
    std::vector<std::filesystem::path> skyboxTexturePaths{
      textureDirectory / "px.png", // right
      textureDirectory / "nx.png", // left
      textureDirectory / "py.png", // top
      textureDirectory / "ny.png", // bottom
      textureDirectory / "pz.png", // front
      textureDirectory / "nz.png"  // back
    };

    Shader skyboxShader{
      std::filesystem::path{"shaders/vertex/skybox.glsl"},
      std::filesystem::path{"shaders/fragment/skybox.glsl"},
      std::vector<std::string>{"skybox", "projection", "view"}
    };

  public:
    Skybox();
    Skybox(const Skybox&) = delete;
    Skybox(Skybox&&) = delete;
    Skybox& operator=(const Skybox&) = delete;
    Skybox& operator=(Skybox&&) = delete;
    ~Skybox();

    void setSkyboxSamplerUniform(Shader::BindObject& shader);
    void setUniformsAndDraw(const Camera& mainCamera);
};
