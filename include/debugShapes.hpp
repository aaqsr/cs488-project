#pragma once

#include "model.hpp"

namespace DebugShape
{

inline Model createCube()
{
    // Create material
    auto material = std::make_shared<Material>();
    material->loadTexture("assets/textures/container.jpg");
    material->setName("SimpleCubeMaterial");
    material->setType(Material::Type::MAT_LAMBERTIAN);
    material->setAmbient({0.1F, 0.1F, 0.1F});
    material->setDiffuse({0.8F, 0.8F, 0.8F});
    material->setSpecular({0.2F, 0.2F, 0.2F});
    material->setSpecularExponent(32.0F);

    // Simple cube vertices (unit cube centered at origin)
    std::vector<Vertex> vertices = {
      // Front face
      { {-0.5F, -0.5F, 0.5F},  {0.0F, 0.0F, 1.0F}, {0.0F, 0.0F}},
      {  {0.5F, -0.5F, 0.5F},  {0.0F, 0.0F, 1.0F}, {1.0F, 0.0F}},
      {   {0.5F, 0.5F, 0.5F},  {0.0F, 0.0F, 1.0F}, {1.0F, 1.0F}},
      {  {-0.5F, 0.5F, 0.5F},  {0.0F, 0.0F, 1.0F}, {0.0F, 1.0F}},

      // Back face
      {{-0.5F, -0.5F, -0.5F}, {0.0F, 0.0F, -1.0F}, {1.0F, 0.0F}},
      { {0.5F, -0.5F, -0.5F}, {0.0F, 0.0F, -1.0F}, {0.0F, 0.0F}},
      {  {0.5F, 0.5F, -0.5F}, {0.0F, 0.0F, -1.0F}, {0.0F, 1.0F}},
      { {-0.5F, 0.5F, -0.5F}, {0.0F, 0.0F, -1.0F}, {1.0F, 1.0F}},

      // Left face
      {{-0.5F, -0.5F, -0.5F}, {-1.0F, 0.0F, 0.0F}, {0.0F, 0.0F}},
      { {-0.5F, -0.5F, 0.5F}, {-1.0F, 0.0F, 0.0F}, {1.0F, 0.0F}},
      {  {-0.5F, 0.5F, 0.5F}, {-1.0F, 0.0F, 0.0F}, {1.0F, 1.0F}},
      { {-0.5F, 0.5F, -0.5F}, {-1.0F, 0.0F, 0.0F}, {0.0F, 1.0F}},

      // Right face
      {  {0.5F, -0.5F, 0.5F},  {1.0F, 0.0F, 0.0F}, {0.0F, 0.0F}},
      { {0.5F, -0.5F, -0.5F},  {1.0F, 0.0F, 0.0F}, {1.0F, 0.0F}},
      {  {0.5F, 0.5F, -0.5F},  {1.0F, 0.0F, 0.0F}, {1.0F, 1.0F}},
      {   {0.5F, 0.5F, 0.5F},  {1.0F, 0.0F, 0.0F}, {0.0F, 1.0F}},

      // Top face
      {  {-0.5F, 0.5F, 0.5F},  {0.0F, 1.0F, 0.0F}, {0.0F, 0.0F}},
      {   {0.5F, 0.5F, 0.5F},  {0.0F, 1.0F, 0.0F}, {1.0F, 0.0F}},
      {  {0.5F, 0.5F, -0.5F},  {0.0F, 1.0F, 0.0F}, {1.0F, 1.0F}},
      { {-0.5F, 0.5F, -0.5F},  {0.0F, 1.0F, 0.0F}, {0.0F, 1.0F}},

      // Bottom face
      {{-0.5F, -0.5F, -0.5F}, {0.0F, -1.0F, 0.0F}, {0.0F, 0.0F}},
      { {0.5F, -0.5F, -0.5F}, {0.0F, -1.0F, 0.0F}, {1.0F, 0.0F}},
      {  {0.5F, -0.5F, 0.5F}, {0.0F, -1.0F, 0.0F}, {1.0F, 1.0F}},
      { {-0.5F, -0.5F, 0.5F}, {0.0F, -1.0F, 0.0F}, {0.0F, 1.0F}}
    };

    // Simple cube indices
    std::vector<uint32_t> indices = {// Front face
                                     0, 1, 2, 2, 3, 0,
                                     // Back face
                                     4, 5, 6, 6, 7, 4,
                                     // Left face
                                     8, 9, 10, 10, 11, 8,
                                     // Right face
                                     12, 13, 14, 14, 15, 12,
                                     // Top face
                                     16, 17, 18, 18, 19, 16,
                                     // Bottom face
                                     20, 21, 22, 22, 23, 20};

    std::vector<Mesh> meshes;
    meshes.emplace_back(std::move(vertices), std::move(indices), material);

    return Model{std::move(meshes),
                 std::unordered_map<std::string, std::shared_ptr<Material>>{
                   {"", material}}};
}

inline Model createRectangle()
{
    auto material = std::make_shared<Material>();
    material->loadTexture("assets/textures/container.jpg");
    material->setName("SimpleCubeMaterial");
    material->setType(Material::Type::MAT_LAMBERTIAN);
    material->setAmbient({0.1F, 0.1F, 0.1F});
    material->setDiffuse({0.8F, 0.8F, 0.8F});
    material->setSpecular({0.2F, 0.2F, 0.2F});
    material->setSpecularExponent(32.0F);

    std::vector<Vertex> vertices = {
      // Bottom-left
      {{-0.7F, -0.5F, 0.0F}, {0.0F, 0.0F, 0.0F}, {1.0F, 0.0F}},
      // Bottom-right
      {{-0.1F, -0.5F, 0.0F}, {0.0F, 0.0F, 0.0F}, {1.0F, 1.0F}},
      // Top-right
      { {-0.1F, 0.5F, 0.0F}, {0.0F, 0.0F, 0.0F}, {0.0F, 1.0F}},
      // Top-left
      { {-0.7F, 0.5F, 0.0F}, {0.0F, 0.0F, 0.0F}, {1.0F, 0.0F}}
    };

    // Indices for two triangles forming a square
    std::vector<unsigned int> indices = {
      0, 1, 2, // First triangle (bottom-left, bottom-right, top-right)
      2, 3, 0  // Second triangle (top-right, top-left, bottom-left)
    };

    std::vector<Mesh> meshes;
    meshes.emplace_back(std::move(vertices), std::move(indices), material);

    return Model{std::move(meshes),
                 std::unordered_map<std::string, std::shared_ptr<Material>>{
                   {"", material}}};
}

} // namespace DebugShape
