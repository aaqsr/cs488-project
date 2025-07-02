#pragma once

#include "material.hpp"
#include "vertex.hpp"

#include <cstdint>
#include <memory>
#include <vector>

class Mesh
{
    uint32_t VAO{0}, VBO{0}, EBO{0};
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    std::shared_ptr<Material> material;

    void setupMesh();
    void setVertices(const std::vector<Vertex>& v);
    void setIndices(const std::vector<uint32_t>& i);

  public:
    Mesh(std::vector<Vertex> vertices, std::vector<uint32_t> indices,
         const std::shared_ptr<Material>& material)
      : vertices{std::move(vertices)}, indices{std::move(indices)},
        material{material}
    {
        setupMesh();
    }

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&&) noexcept;
    Mesh& operator=(Mesh&&) noexcept;

    ~Mesh();

    // CALLER IS EXPECTED TO HAVE BOUND THE SHADER
    // TODO: Does not check if shader is bound!
    void draw(Shader& shader) const;
};
