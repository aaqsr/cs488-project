#pragma once

#include "mesh.hpp"
#include "util/logger.hpp"
#include "util/quaternion.hpp"

#include "linalg.h"
#include <filesystem>
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
    linalg::aliases::float4x4 modelMatrix = linalg::identity;

    // TEMPORARILY PUBLIC FOR TESTING ONLY
    // public:
    //   linalg::aliases::float3 worldPos = {0.0F, 0.0F, 0.0F};
    //   linalg::aliases::float3 scale = {1.0F, 1.0F, 1.0F};
    //   Quaternion rotation{};

    // private:

    void updateModelMatrix(const linalg::aliases::float3& worldPos,
                           const Quaternion& rotation,
                           const linalg::aliases::float3& scale = {1.0F, 1.0F,
                                                                   1.0F});

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
