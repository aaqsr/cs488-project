#pragma once

#include "logger.hpp"
#include "mesh.hpp"
#include "quaternion.hpp"

#include <filesystem>
#include <linalg.h>
#include <utility>
#include <vector>

class Material;
class Mesh;
class Shader;

class Model
{
    std::filesystem::path objPath;
    std::vector<Mesh> meshes;
    std::unordered_map<std::string, std::shared_ptr<Material>> materials;

    // Model matrix = translate * rotate * scale
    linalg::aliases::float4x4 modelMatrix;

    // TEMPORARILY PUBLIC FOR TESTING ONLY
  public:
    linalg::aliases::float3 worldPos = {0.0F, 0.0F, 0.0F};
    linalg::aliases::float3 scale = {1.0F, 1.0F, 1.0F};
    Quaternion rotation{};

  private:
    void updateModelMatrix();

    void loadModel();
    void loadMaterials(const std::filesystem::path& mtlPath);

  public:
    Model(std::vector<Mesh> meshes,
          std::unordered_map<std::string, std::shared_ptr<Material>> materials)
      : meshes{std::move(meshes)}, materials{std::move(materials)}
    {
        // Logger::GetInstance().log("DO NOT BUILD MODEL BY HAND");
    }

    explicit Model(std::filesystem::path objPath);

    Model(const Model&) = delete;
    Model& operator=(const Model&) = delete;
    Model(Model&&) = default;
    Model& operator=(Model&&) = default;
    ~Model() = default;

    // USER MUST BIND SHADER
    void updateModelMatrixAndDraw(Shader::BindObject& shader);
};
