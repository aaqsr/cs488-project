#include "renderer.hpp"
#include "debugShapes.hpp"
#include "mesh.hpp"
#include "shader.hpp"

#include <numbers>

Renderer::Renderer()
{
    //
    // OpenGL configuration
    //
    glEnable(GL_CULL_FACE);  // don't draw back faces
    glEnable(GL_DEPTH_TEST); // Depth buffer
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE); // allow writing to depth buffer
    glDepthRange(0.0, 1.0);
    glClearDepth(1.0); // clear depth buffer to 1.0 (far plane)

    // Disable VSync for them FPS wooooo (TODO: good idea??)
    glfwSwapInterval(0);

    //
    // Setting up sub-classes
    //
    controller.setMainCamera(&mainCamera);
}

void Renderer::drawLoop()
{
    // TODO: should be easier to do than one object
    // TODO: should be easier to do different shaders without everyone assigning
    // things that don't exist
    // TODO: make it so that we can call UseShader on a shader once before
    // all calls to it
    // TODO: should be less easy to forget to do this correctly
    {
        Shader::BindObject boundShader = shader.bind();
        mainCamera.setUniforms(boundShader);
        light.setUniforms(boundShader, 0);
        mainModel.draw(boundShader);
    }

    {
        Shader::BindObject boundShader = flatShader.bind();
        mainCamera.setUniforms(boundShader);
        static Model lcube = DebugShape::createCube(light.getPos(), 0.1F);
        lcube.draw(boundShader);
    }
}

void Renderer::loop()
{
    while (!window.shouldClose()) {
        //  Calculate delta time
        currentFrameTime = glfwGetTime();
        deltaTime = currentFrameTime - lastFrameTime;
        lastFrameTime = currentFrameTime;

        // performance metrics
        msPerFrame.tick(currentFrameTime);

        controller.update(deltaTime);

        // not clearing the back buffer causes trails
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.2F, 0.3F, 0.3F, 1.0F); // set a colour background

        /* RENDER COMMANDS HERE */

        drawLoop();

        // Enable this for Wireframe mode
        // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

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
