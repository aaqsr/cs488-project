#include "camera.hpp"
#include "mesh.hpp"
#include "shader.hpp"
#include "vertex.hpp"
#include "window.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <exception>
#include <iostream>
#include <stdexcept>
#include <vector>

void run()
{
    Window& window = Window::GetInstance();

    Shader rainbowShader{
      std::filesystem::path{"shaders/vertex/perspectiveRainbowTriTest.glsl"},
      std::filesystem::path{"shaders/fragment/rainbowTriTest.glsl"},
      {"projection", "view"}
    };

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


    // Indices for two triangles forming a square
    std::vector<unsigned int> indices1 = {
      0, 1, 2, // First triangle (bottom-left, bottom-right, top-right)
      2, 3, 0  // Second triangle (top-right, top-left, bottom-left)
    };

    Mesh mesh;

    mesh.setVertices(vertices1);
    mesh.setIndices(indices1);
    mesh.setupMesh();

    Camera camera;

    while (!window.shouldClose()) {
        // not clearing the back buffer causes trails
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        /* RENDER COMMANDS HERE */

        // Enable this for Wireframe mode
        // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        camera.setUniforms(rainbowShader);

        rainbowShader.use();
        mesh.draw();

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

int main()
{
    try {
        run();
    } catch (std::runtime_error& e) {
        std::cout << e.what() << "\n";
        return 1;
    }
}
