#pragma once

#include "mesh.hpp"
#include <filesystem>
#include <iostream>
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

    void loadModel();
    void loadMaterials(const std::filesystem::path& mtlPath);

  public:
    Model(std::vector<Mesh> meshes,
          std::unordered_map<std::string, std::shared_ptr<Material>> materials)
      : meshes{std::move(meshes)}, materials{std::move(materials)}
    {
        std::cout << "DO NOT BUILD MODEL BY HAND\n";
    }

    explicit Model(std::filesystem::path objPath);

    Model(const Model&) = delete;
    Model& operator=(const Model&) = delete;
    Model(Model&&) = default;
    Model& operator=(Model&&) = default;
    ~Model() = default;

    void draw(Shader& shader) const;
};
