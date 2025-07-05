#include "frontend/model.hpp"
#include "frontend/material.hpp"
#include "frontend/mesh.hpp"
#include "frontend/shader.hpp"
#include "util/error.hpp"
#include "util/logger.hpp"

#include "linalg.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>

Model::Model(std::filesystem::path objPath) : objPath{std::move(objPath)}
{
    loadModel();
}

void Model::updateModelMatrix()
{
    modelMatrix = linalg::mul(
      linalg::mul(linalg::translation_matrix(worldPos), rotation.toMatrix4x4()),
      linalg::scaling_matrix(scale));
}

void Model::updateModelMatrixAndDraw(Shader::BindObject& shader)
{
    // TODO: Model matrix probably does not need to be recomputed every frame,
    // only when something changes. Add a way to cache values.
    updateModelMatrix();
    shader.setUniform("model", modelMatrix);

    for (const auto& mesh : meshes) {
        mesh.draw(shader);
    }
}

void Model::loadModel()
{
    std::ifstream file(objPath);
    if (!file.is_open()) {
        throw IrrecoverableError{"Failed to open OBJ file: " +
                                 objPath.string()};
    }

    std::vector<linalg::aliases::float3> positions;
    std::vector<linalg::aliases::float3> normals;
    std::vector<linalg::aliases::float2> texCoords;
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    std::string currentMaterialName;

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;

        float x = 0;
        float y = 0;
        float z = 0;
        float u = 0;
        float v = 0;

        if (type == "mtllib") {
            std::string mtlFile;
            iss >> mtlFile;
            loadMaterials(objPath.parent_path() / mtlFile);
        } else if (type == "v") {
            iss >> x >> y >> z;
            positions.emplace_back(x, y, z);
        } else if (type == "vt") {
            iss >> u >> v;
            texCoords.emplace_back(u, v);
        } else if (type == "vn") {
            iss >> x >> y >> z;
            normals.emplace_back(x, y, z);
        } else if (type == "usemtl") {
            if (!vertices.empty() && !currentMaterialName.empty() &&
                materials.contains(currentMaterialName))
            {
                meshes.emplace_back(vertices, indices,
                                    materials[currentMaterialName]);
                vertices.clear();
                indices.clear();
            }
            iss >> currentMaterialName;
            // if the material is not loaded, warn or fallback
            if (!materials.contains(currentMaterialName)) {
                // fallback: create a default material if not found
                materials[currentMaterialName] = std::make_shared<Material>();
            }
        } else if (type == "f") {
            std::string vertex;
            std::vector<uint32_t> faceIndices;
            while (iss >> vertex) {
                std::istringstream vertexStream(vertex);
                std::string indexStr;
                std::vector<int> vertexIndices;
                while (std::getline(vertexStream, indexStr, '/')) {
                    if (indexStr.empty()) {
                        vertexIndices.push_back(0);
                    } else {
                        vertexIndices.push_back(std::stoi(indexStr));
                    }
                }
                int posIndex = vertexIndices[0] - 1;
                int texIndex =
                  vertexIndices.size() > 1 ? vertexIndices[1] - 1 : -1;
                int normIndex =
                  vertexIndices.size() > 2 ? vertexIndices[2] - 1 : -1;
                Vertex v;
                v.position = positions[posIndex];
                v.texCoords = texIndex >= 0
                                ? texCoords[texIndex]
                                : linalg::aliases::float2{0.0F, 0.0F};
                v.normal = normIndex >= 0
                             ? normals[normIndex]
                             : linalg::aliases::float3{0.0F, 1.0F, 0.0F};
                auto it = std::ranges::find(vertices, v);
                uint32_t index = 0;
                if (it == vertices.end()) {
                    index = static_cast<uint32_t>(vertices.size());
                    vertices.push_back(v);
                } else {
                    index = static_cast<uint32_t>(
                      std::distance(vertices.begin(), it));
                }
                faceIndices.push_back(index);
            }
            for (size_t i = 2; i < faceIndices.size(); ++i) {
                indices.push_back(faceIndices[0]);
                indices.push_back(faceIndices[i - 1]);
                indices.push_back(faceIndices[i]);
            }
        }
    }
    // add last mesh if there are remaining vertices and a valid material
    if (!vertices.empty() && !currentMaterialName.empty() &&
        materials.contains(currentMaterialName))
    {
        meshes.emplace_back(vertices, indices, materials[currentMaterialName]);
    }

    std::cout << "Loaded " << meshes.size() << " meshes\n";
    std::cout << "First mesh has " << vertices.size() << " vertices\n";
}

void Model::loadMaterials(const std::filesystem::path& mtlPath)
{
    std::ifstream file(mtlPath);
    if (!file.is_open()) {
        throw IrrecoverableError("Failed to open MTL file: " +
                                 mtlPath.string());
    }
    std::string currentMaterialName;
    std::shared_ptr<Material> currentMaterial;
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        if (type == "newmtl") {
            if (currentMaterial && !currentMaterialName.empty()) {
                materials[currentMaterialName] = currentMaterial;
            }
            iss >> currentMaterialName;
            currentMaterial = std::make_shared<Material>();
        } else if (currentMaterial) {
            float r = 0;
            float g = 0;
            float b = 0;
            float ns = 0;

            if (type == "Ka") {
                iss >> r >> g >> b;
                currentMaterial->setAmbient({r, g, b});
            } else if (type == "Kd") {
                iss >> r >> g >> b;
                currentMaterial->setDiffuse({r, g, b});
            } else if (type == "Ks") {
                iss >> r >> g >> b;
                currentMaterial->setSpecular({r, g, b});
            } else if (type == "Ns") {
                iss >> ns;
                currentMaterial->setSpecularExponent(ns);
            } else if (type == "map_Kd") {
                std::string texPath;
                iss >> texPath;
                currentMaterial->loadDiffuseMap(mtlPath.parent_path() /
                                                texPath);
            } else if (type == "map_Ks") {
                std::string texPath;
                iss >> texPath;
                currentMaterial->loadSpecularMap(mtlPath.parent_path() /
                                                 texPath);
            } else {
                Logger::GetInstance().log(
                  "Model Loading >>  mtl file >> Unidentified type: " + type);
            }
        }
    }
    if (currentMaterial && !currentMaterialName.empty()) {
        materials[currentMaterialName] = currentMaterial;
    }
}
