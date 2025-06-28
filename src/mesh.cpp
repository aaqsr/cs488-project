#include "mesh.hpp"

#include "vertex.hpp"

#include <GL/glew.h>

void Mesh::setVertices(const std::vector<Vertex>& v)
{
    vertices = v;
}

void Mesh::setupMesh()
{

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

void Mesh::draw() const
{
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()),
                   GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
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
