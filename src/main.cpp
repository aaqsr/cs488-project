#include "camera.hpp"
#include "controller.hpp"
#include "debugShapes.hpp"
#include "mesh.hpp"
#include "model.hpp"
#include "shader.hpp"
#include "vertex.hpp"
#include "window.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "error.hpp"
#include <filesystem>
#include <iostream>
#include <vector>

namespace
{

void run()
{
    // TODO: Window MUST be init first... should really make a renderer class
    // that makes it so that I can't accidentally not initialise a window...
    Window& window = Window::GetInstance();
    Controller& controller = Controller::GetInstance();

    Shader texShader{
      std::filesystem::path{"shaders/vertex/perspTextureShader.glsl"},
      std::filesystem::path{"shaders/fragment/textureShader.glsl"},
      {"projection", "view"}
    };

    Model teapot{std::filesystem::path{"assets/models/teapot/teapot.obj"}};
    // Model cube = DebugShape::createCube();

    Camera camera;
    controller.setMainCamera(&camera);

    float lastFrameTime = glfwGetTime();

    float lastTimeFramePerformanceComputed = glfwGetTime();
    int nbFrames = 0;

    while (!window.shouldClose()) {
        //  Calculate delta time
        float currentFrameTime = static_cast<float>(glfwGetTime());
        float deltaTime = currentFrameTime - lastFrameTime;
        lastFrameTime = currentFrameTime;

        nbFrames++;
        if (currentFrameTime - lastTimeFramePerformanceComputed >= 1.0F) {
            printf("%f ms/frame\n", 1000.0 / double(nbFrames));
            nbFrames = 0;
            lastTimeFramePerformanceComputed += 1.0;
        }

        controller.update(deltaTime);

        // not clearing the back buffer causes trails
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.2F, 0.3F, 0.3F, 1.0F); // Dark gray background

        /* RENDER COMMANDS HERE */

        // Enable this for Wireframe mode
        // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        // TODO: Someone else should be doing this I feel
        // TODO: MUST RESTRUCTURE THE WAY THIS MAIN LOOP IS DONE. TOO MANY
        // FOOTGUNS LURKING ABOUT.
        // TODO: Must restructure for the following
        // Minimize per-frame data transfers from CPU to GPU
        // Minimize number of state changes (binding framebuffers, textures,
        // shaders, buffers, etcetera)
        // Minimize number of individual draw calls per frame

        camera.setUniforms(texShader);

        teapot.draw(texShader);

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
} // namespace

int main()
{
    try {
        run();
    } catch (IrrecoverableError& e) {
        std::cout << e.msg << "\n" << e.what() << "\n";
        return 1;
    }
}
