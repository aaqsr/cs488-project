#include "vertex.hpp"

#include <GL/glew.h>

// TODO: can we somehow generate this boilerplate?
void Vertex::setupVertexAttributes()
{
    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void*)offsetof(Vertex, position));
    glEnableVertexAttribArray(0);

    // Color attribute
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void*)offsetof(Vertex, colour));
    glEnableVertexAttribArray(1);
}
