#include "mesh.hpp"

#include "error.hpp"
#include "vertex.hpp"

#include <GL/glew.h>

void Mesh::setVertices(const std::vector<Vertex>& v)
{
    vertices = v;
}
void Mesh::setIndices(const std::vector<uint32_t>& i)
{
    indices = i;
}

void Mesh::setupMesh()
{
    if (material == nullptr) {
        throw IrrecoverableError{"Mesh material was nullptr"};
    }

    // Generate and bind VAO
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);

    // Generate and bind VBO
    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)),
                 vertices.data(), GL_STATIC_DRAW);

    // Generate and bind EBO
    glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(indices.size() * sizeof(unsigned int)),
                 indices.data(), GL_STATIC_DRAW);

    Vertex::setupVertexAttributes();

    // Unbind VAO
    glBindVertexArray(0);
}

void Mesh::draw(Shader::BindObject& shader) const
{
    glBindVertexArray(VAO);

    material->setUniformsAndBind(shader);

    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()),
                   GL_UNSIGNED_INT, nullptr);

    glBindVertexArray(0);
}

Mesh::Mesh(Mesh&& other) noexcept
  : VAO{other.VAO}, VBO{other.VBO}, EBO{other.EBO},
    vertices{std::move(other.vertices)}, indices{std::move(other.indices)},
    material{std::move(other.material)}
{
    other.VAO = 0;
    other.VBO = 0;
    other.EBO = 0;
}

Mesh& Mesh::operator=(Mesh&& other) noexcept
{
    // TODO: wait what why am i doing this, i should just be swapping the two...
    if (this != &other) {
        if (VAO != 0) {
            glDeleteVertexArrays(1, &VAO);
        }
        if (VBO != 0) {
            glDeleteBuffers(1, &VBO);
        }
        if (EBO != 0) {
            glDeleteBuffers(1, &EBO);
        }

        VAO = other.VAO;
        other.VAO = 0;
        VBO = other.VBO;
        other.VBO = 0;
        EBO = other.EBO;
        other.EBO = 0;

        vertices = std::move(other.vertices);
        indices = std::move(other.indices);
        material = std::move(other.material);
    }
    return *this;
}

Mesh::~Mesh()
{
    if (VAO != 0) {
        glDeleteVertexArrays(1, &VAO);
    }
    if (VBO != 0) {
        glDeleteBuffers(1, &VBO);
    }
    if (EBO != 0) {
        glDeleteBuffers(1, &EBO);
    }
}
