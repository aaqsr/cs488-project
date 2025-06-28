#pragma once

#include <cstdint>
#include <vector>

class Vertex;

class Mesh
{
  public:
    Mesh() = default;

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&&) = delete;
    Mesh& operator=(Mesh&&) = delete;

    ~Mesh();

    void setVertices(const std::vector<Vertex>& v);

    void setIndices(const std::vector<uint32_t>& i)
    {
        indices = i;
    }

    void setupMesh();
    void draw() const;

  private:
    uint32_t VAO{0}, VBO{0}, EBO{0};
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
};
