#include "frontend/renderer.hpp"
#include "bridgeChannelData.hpp"
#include "frontend/crosshair.hpp"
#include "frontend/debugShapes.hpp"
#include "frontend/shader.hpp"
#include "linalg.h"
#include "physics/physicsEngine.hpp"
#include "util/channel.hpp"
#include "util/error.hpp"
#include <memory>

std::vector<PhysicsEngineReceiverData> Renderer::createDefaultScene()
{
    std::vector<PhysicsEngineReceiverData> objects;

    // Dimensions and density and weight accurate to a real full glass bottle!
    PhysicsEngineReceiverData bottle{
      .model = std::make_unique<Model>(
        std::filesystem::path{"assets/models/bottle/bottle.obj"}
        ),
      .scale = linalg::aliases::float3{0.045F},
      .initPos = linalg::aliases::float3{1.0F, 1.0F, 2.0F},
      .initVel = linalg::aliases::float3{0.0F, 0.0F, 0.0F},
      .initAngVel = linalg::aliases::float3{0.0F, 0.0F, 0.0F},
      .density = 900.0F
    };
    objects.emplace_back(std::move(bottle));

    // Dimensions and density and weight accurate to a real empty glass bottle!
    PhysicsEngineReceiverData emptyBottle{
      .model = std::make_unique<Model>(
        std::filesystem::path{"assets/models/bottle/bottle.obj"}
        ),
      .scale = linalg::aliases::float3{0.045F},
      .initPos = linalg::aliases::float3{1.0F, 1.0F, 1.0F},
      .initVel = linalg::aliases::float3{0.0F, 0.0F, 0.0F},
      .initAngVel = linalg::aliases::float3{0.0F, 0.0F, 0.0F},
      .density = 300.0F
    };
    objects.emplace_back(std::move(emptyBottle));

    PhysicsEngineReceiverData mrGooseLight{
      .model = std::make_unique<Model>(
        std::filesystem::path{"assets/models/goose/Mesh_Goose.obj"}
        ),
      .scale = linalg::aliases::float3{0.007F},
      .initPos = linalg::aliases::float3{1.0F, 1.0F, 3.0F},
      .initVel = linalg::aliases::float3{0.0F, 0.0F, 0.0F},
      .initAngVel = linalg::aliases::float3{0.0F, 0.0F, 0.0F},
      .density = 700.0F
    };
    objects.emplace_back(std::move(mrGooseLight));

    PhysicsEngineReceiverData mrGooseHeavy{
      .model = std::make_unique<Model>(
        std::filesystem::path{"assets/models/goose/Mesh_Goose.obj"}
        ),
      .scale = linalg::aliases::float3{0.007F},
      .initPos = linalg::aliases::float3{3.0F, 1.0F, 1.0F},
      .initVel = linalg::aliases::float3{0.0F, 0.0F, 0.0F},
      .initAngVel = linalg::aliases::float3{0.0F, 0.0F, 0.0F},
      .density = 1500.0F
    };
    objects.emplace_back(std::move(mrGooseHeavy));

    // PhysicsEngineReceiverData flyingBottle{
    //   .model = std::make_unique<Model>(
    //     std::filesystem::path{"assets/models/bottle/bottle.obj"}
    //     ),
    //   .scale = linalg::aliases::float3{0.045F},
    //   .initPos = linalg::aliases::float3{3.0F, 2.0F, 3.0F},
    //   .initVel = linalg::aliases::float3{0.0F, 7.0F, 0.0F},
    //   .initAngVel = linalg::aliases::float3{5.0F, 2.0F, 0.0F},
    //   .density = 700.0F
    // };
    // objects.emplace_back(std::move(flyingBottle));

    // https://poly.pizza/m/75h3mi6uHuC
    // PhysicsEngineReceiverData car{
    //   .model = std::make_unique<Model>(
    //     std::filesystem::path{"assets/models/car/car.obj"}
    //     ),
    //   .scale = linalg::aliases::float3{0.008F},
    //   .initPos = linalg::aliases::float3{2.5F, 2.0F, 2.0F},
    //   .initVel = linalg::aliases::float3{0.0F, 10.0F, 0.0F},
    //   .initAngVel = linalg::aliases::float3{5.0F, 2.0F, 0.0F},
    //   .density = 800.0F
    // };
    // objects.emplace_back(std::move(car));

    // PhysicsEngineReceiverData emptyPlasticBottle{
    //   .model = std::make_unique<Model>(
    //     std::filesystem::path{"assets/models/bottle/bottle.obj"}
    //     ),
    //   .scale = linalg::aliases::float3{0.045F},
    //   .initPos = linalg::aliases::float3{2.0F, 2.0F, 2.0F},
    //   .initVel = linalg::aliases::float3{0.0F, 0.0F, 0.0F},
    //   .initAngVel = linalg::aliases::float3{3.0F, 0.0F, 0.0F},
    //   .density = 50.0F
    // };
    // msg.getWriteBuffer().emplace_back(std::move(emptyPlasticBottle));

    // PhysicsEngineReceiverData spinningBottle{
    //   .model = std::make_unique<Model>(
    //     std::filesystem::path{"assets/models/bottle/bottle.obj"}
    //     ),
    //   .scale = linalg::aliases::float3{0.045F},
    //   .initPos = linalg::aliases::float3{1.0F, 1.0F, 0.0F},
    //   .initVel = linalg::aliases::float3{5.0F, 10.0F, 0.0F},
    //   .initAngVel = linalg::aliases::float3{5.0F, 0.0F, 0.0F},
    //   .density = 20.0F
    // };
    // msg.getWriteBuffer().emplace_back(std::move(spinningBottle));

    // PhysicsEngineReceiverData debugCube{
    //   .model = std::make_unique<Model>(DebugShape::createCube()),
    //   .scale = linalg::aliases::float3{0.4F},
    //   .initPos = linalg::aliases::float3{3.0F, 2.0F, 3.0F},
    //   .initVel = linalg::aliases::float3{1.0F, 10.0F, 0.0F},
    //   .initAngVel = linalg::aliases::float3{5.0F, 2.0F, 0.0F},
    //   .density = 700.0F
    // };
    // msg.getWriteBuffer().emplace_back(std::move(debugCube));

    return objects;
}

void Renderer::init()
{
    if (bridgeChannel == nullptr || physCmdChannel == nullptr) {
        throw IrrecoverableError{
          "Render::init() called without setting required channels"};
    }

    // load scene objects from JSON file
    std::vector<PhysicsEngineReceiverData> physicsObjects;

    if (!sceneFilePath.empty()) {
        try {
            auto sceneObjects = Scene::SceneLoader::loadFromFile(sceneFilePath);
            physicsObjects =
              Scene::SceneLoader::convertToPhysicsObjects(sceneObjects);
        } catch (const std::exception& e) {
            throw IrrecoverableError{"Failed to load scene file '" +
                                     sceneFilePath.string() + "': " + e.what()};
        }
    } else {
        // fallback to hardcoded objects if no scene file provided
        physicsObjects = createDefaultScene();
    }

    // send scene physics objects to physics engine
    {
        auto msg = physCmdChannel->createMessage();
        for (auto& physObj : physicsObjects) {
            msg.getWriteBuffer().emplace_back(std::move(physObj));
        }
    }

    {
        Shader::BindObject shader = waterShader.bind();
        skybox.setSkyboxSamplerUniform(shader);
    }

    {
        Shader::BindObject shader = waterShader.bind();
        skybox.setSkyboxSamplerUniform(shader);
    }
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

    {
        Shader::BindObject boundShader = flatShader.bind();
        mainCamera.setUniforms(boundShader);
        pool.draw(boundShader);
    }

    {
        bool isThisNewData = bridgeChannel->isMessageReady();
        auto message = bridgeChannel->receive();

        {
            // Check for splash conditions and emit particles (only when
            // simulation is running)
            if (isThisNewData) {
                for (const auto& rigidBody : message.getBuffer().physicsObjects)
                {
                    if (!rigidBody.enabled) {
                        continue;
                    }

                    linalg::aliases::float3 position =
                      rigidBody.getWorldPosition();
                    linalg::aliases::float3 velocity =
                      rigidBody.getLinearVelocity();

                    // Simple splash detection: check if object is moving fast
                    // and near water surface
                    float velocityMagnitude = linalg::length(velocity);
                    if (velocityMagnitude > 1.0F) {
                        // Check if near water surface
                        if (WaterSimulation::isPositionInWater(
                              position, message.getBuffer().waterHeights))
                        {
                            float intensity =
                              std::min(0.8F, velocityMagnitude / 5.0F);
                            particleSystem.emitSplash(position, velocity,
                                                      intensity);
                        }
                    }
                }
            }

            // Update particles only when simulation is running
            // Check if we received new data to determine if simulation is
            // running
            float simulationDeltaTime = isThisNewData ? deltaTime : 0.0;

            particleSystem.update(
              simulationDeltaTime,
              [&message](const linalg::aliases::float3& pos) -> float {
                  if (!WaterSimulation::isPositionInWater(
                        pos, message.getBuffer().waterHeights))
                  {
                      return 0.0F;
                  }

                  size_t i = static_cast<size_t>(
                    (pos.z - WaterSimulation::bottomLeftCornerWorldPos_xz.y) /
                    WaterSimulation::cellSize);
                  size_t j = static_cast<size_t>(
                    (pos.x - WaterSimulation::bottomLeftCornerWorldPos_xz.x) /
                    WaterSimulation::cellSize);

                  if (i >= WaterSimulation::numRows ||
                      j >= WaterSimulation::numCols)
                  {
                      return 0.0F;
                  }

                  return message.getBuffer().waterHeights.getWaterHeight(i, j);
              });
        }

        {
            Shader::BindObject boundShader = sunShader.bind();
            mainCamera.setUniforms(boundShader);
            for (const auto& rigidBody : message.getBuffer().physicsObjects) {
                if (!rigidBody.enabled) {
                    continue;
                }

                Model& m = *(rigidBody.getModelPtr());
                m.updateModelMatrix(rigidBody.getWorldPosition(),
                                    rigidBody.getOrientation(),
                                    rigidBody.getScale());
                m.draw(boundShader);
            }
        }

        // Should be second last thing drawn
        {
            skybox.setUniformsAndDraw(mainCamera);
        }

        if (DEBUGMODE) {
            aabbs.clear();
            for (const auto& rigidBody : message.getBuffer().physicsObjects) {
                aabbs.emplace_back(rigidBody.computeAABB());
            }
            aabbVisualiser.draw(aabbs, mainCamera.getViewMatrix(),
                                mainCamera.getPerspectiveMatrix());
        }

        // Should be last thing drawn
        {
            Shader::BindObject boundShader = waterShader.bind();
            mainCamera.setUniforms(boundShader);

            if (isThisNewData) {
                waterMesh.updateMesh(
                  message.getBuffer().waterHeights.getWaterHeights());
            }

            waterMesh.draw(boundShader, mainCamera.getPosition());
        }

        // render particles
        if (particleSystem.getAliveCount() > 0) {
            particleRenderer.draw(particleSystem, mainCamera);
        }
    }
}

void Renderer::loop()
{
    if (bridgeChannel == nullptr || physCmdChannel == nullptr) {
        throw IrrecoverableError{
          "Render::loop() called without setting required channels"};
    }

    while (!window.shouldClose()) {
        //  Calculate delta time
        currentFrameTime = glfwGetTime();
        deltaTime = currentFrameTime - lastFrameTime;
        lastFrameTime = currentFrameTime;

        // performance metrics
        msPerFrame.tick();

        controller.update(deltaTime);

        // not clearing the back buffer causes trails
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.2F, 0.3F, 0.3F, 1.0F); // set a colour background

        /* RENDER COMMANDS HERE */
        update();

        HUD::GetInstance().draw();

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
    glfwSwapInterval(1);

    //
    // Setting up sub-classes
    //
    controller.setMainCamera(&mainCamera);
}

void Renderer::attachBridgeChannel(Receiver<BridgeChannelData>* r)
{
    bridgeChannel = r;
}

void Renderer::attachPhysicsEngineCommandsChannel(
  Sender<std::vector<PhysicsEngineReceiverData>>* s)
{
    physCmdChannel = s;
    controller.setPhysicsCommandChannel(physCmdChannel);
}

void Renderer::toggleDebugMode()
{
    Logger::GetInstance().log("Toggled Debug Mode");
    DEBUGMODE = !DEBUGMODE;
}

void Renderer::setSceneFile(const std::filesystem::path& scenePath)
{
    sceneFilePath = scenePath;
}
