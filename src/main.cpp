#include "mesh.hpp"
#include "shader.hpp"
#include "vertex.hpp"
#include "window.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <vector>

int main()
{
    Window& window = Window::GetInstance();

    Shader rainbowShader{
      std::filesystem::path{"shaders/vertex/rainbowTriTest.glsl"},
      std::filesystem::path{"shaders/fragment/rainbowTriTest.glsl"}};
    Shader flatColourShader{
      std::filesystem::path{"shaders/vertex/flatTriTest.glsl"},
      std::filesystem::path{"shaders/fragment/flatTriTest.glsl"}};

    std::vector<Vertex> vertices1 = {
      // Bottom-left
      {{-0.7F, -0.5F, 0.0F}, {1.0F, 0.0F, 0.0F, 1.0F}}, // Red
      // Bottom-right
      {{-0.1F, -0.5F, 0.0F}, {0.0F, 1.0F, 0.0F, 1.0F}}, // Green
      // Top-right
      { {-0.1F, 0.5F, 0.0F}, {0.0F, 0.0F, 1.0F, 1.0F}}, // Blue
      // Top-left
      { {-0.7F, 0.5F, 0.0F}, {1.0F, 1.0F, 0.0F, 1.0F}}  // Yellow
    };

    std::vector<Vertex> vertices2 = {
      // Bottom-left
      {{0.2F, -0.5F, 0.0F}, {1.0F, 0.0F, 0.0F, 1.0F}}, // Red
      // Bottom-right
      {{0.8F, -0.5F, 0.0F}, {0.0F, 1.0F, 0.0F, 1.0F}}, // Green
      // Top
      { {0.5F, 0.5F, 0.0F}, {0.0F, 0.0F, 1.0F, 1.0F}}, // Blue
    };

    // Indices for two triangles forming a square
    std::vector<unsigned int> indices1 = {
      0, 1, 2, // First triangle (bottom-left, bottom-right, top-right)
      2, 3, 0  // Second triangle (top-right, top-left, bottom-left)
    };

    // indices for second triangle
    std::vector<unsigned int> indices2 = {
      0, 1, 2, // First triangle (bottom-left, bottom-right, top-right)
    };

    Mesh mesh;
    Mesh mesh2;

    mesh.setVertices(vertices1);
    mesh.setIndices(indices1);
    mesh.setupMesh();

    mesh2.setVertices(vertices2);
    mesh2.setIndices(indices2);
    mesh2.setupMesh();

    while (!window.shouldClose()) {
        // not clearing the back buffer causes trails
        glClear(GL_COLOR_BUFFER_BIT);

        /* RENDER COMMANDS HERE */

        // Enable this for Wireframe mode
        // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        rainbowShader.use();
        mesh.draw();

        flatColourShader.use();
        mesh2.draw();

        // Double buffering.
        // The front buffer contains the final output image that is
        // shown at the screen. Whilst all the rendering commands
        // draw to the back buffer. We swap the back buffer to the
        // front buffer so the image can be displayed without still
        // being rendered to.
        window.swapBuffers();
        glfwPollEvents();
    }
}
