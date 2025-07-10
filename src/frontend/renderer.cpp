#include "frontend/renderer.hpp"
#include "frontend/debugShapes.hpp"
#include "frontend/pointLight.hpp"
#include "frontend/shader.hpp"
#include "util/quaternion.hpp"
#include <numbers>

void Renderer::init()
{
    mainModel.rotation =
      Quaternion::fromEulerAngles(-std::numbers::pi / 2, 0.0F, 0.0F);
    mainModel.scale = {0.03F, 0.03F, 0.03F};
    mainModel.worldPos = {0.0F, -0.22F, 0.0F};

    teaPot2.worldPos = {-2.0F, 0.0F, 0.0F};
}

void Renderer::update()
{
    // TODO: should be easier to do than one object
    // TODO: should be easier to do different shaders without everyone assigning
    // things that don't exist
    // TODO: make it so that we can call UseShader on a shader once before
    // all calls to it
    // TODO: should be less easy to forget to do this correctly

    // {
    //     Shader::BindObject boundShader = shader.bind();
    //     mainCamera.setUniforms(boundShader);
    //     light.setUniforms(boundShader, 0);
    //     mainModel.updateModelMatrixAndDraw(boundShader);
    // }

    // {
    //     Shader::BindObject boundShader = shader.bind();
    //     mainCamera.setUniforms(boundShader);
    //     light.setUniforms(boundShader, 0);
    //     mainModel.updateModelMatrixAndDraw(boundShader);
    // }
    //
    // {
    //     Shader::BindObject boundShader = flatShader.bind();
    //     teaPot2.updateModelMatrixAndDraw(boundShader);
    //     mainCamera.setUniforms(boundShader);
    //     static Model lcube =
    //     DebugShape::createCubeWithDefaultModelMatrix(light.getPos(), 0.1F);
    //     lcube.updateModelMatrixAndDraw(boundShader);
    // }

    {
        Shader::BindObject boundShader = flatShader.bind();
        mainCamera.setUniforms(boundShader);
        // light.setUniforms(boundShader, 0);
        //
        // const double velocity = 3.0 * deltaTime;
        //
        static Model lcube = DebugShape::createCube();
        lcube.worldPos = {-2.0F, 0.0F, 0.0F};
        lcube.updateModelMatrixAndDraw(boundShader);
        //
        //     static Model lcubeScale = DebugShape::createCube();
        //     static float scaleParam = 0.0F;
        //     lcubeScale.worldPos = {1.0F, 0.0F, -2.0F};
        //     lcubeScale.scale = {0.5F * sin(scaleParam) + 1.0F, 1.0F, 0.5F *
        //     cos(scaleParam) + 1.0F}; scaleParam += velocity;
        //     lcubeScale.updateModelMatrixAndDraw(boundShader);
        //
        //     static Model lcubeRot = DebugShape::createCube();
        //
        //     lcubeRot.updateModelMatrixAndDraw(boundShader);
        //
        //     static float Xscale = 0.0F;
        //     static float Zscale = 0.0F;
        //
        //     static float rotAxParam = 0.0F;
        //     linalg::aliases::float3 rotationAxis = {sin(rotAxParam),
        //     cos(rotAxParam), 0.0F};
        //
        //     rotAxParam += velocity;
        //     Quaternion incrementalRotation =
        //       Quaternion::fromAxisAngle(rotationAxis, velocity);
        //     lcubeRot.rotation = (incrementalRotation *
        //     lcubeRot.rotation).normalized();
    }

    {
        Shader::BindObject boundShader = waterShader.bind();
        mainCamera.setUniforms(boundShader);
        sim.update();
        sim.draw(boundShader);
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
        update();

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

    // glEnable(GL_PROGRAM_POINT_SIZE);
    glPointSize(5.0F);

    // Disable VSync for them FPS wooooo (TODO: good idea??)
    glfwSwapInterval(0);

    //
    // Setting up sub-classes
    //
    controller.setMainCamera(&mainCamera);
    controller.setSim(&sim);
}
