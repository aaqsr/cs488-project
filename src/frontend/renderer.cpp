#include "frontend/renderer.hpp"
#include "bridgeChannelData.hpp"
#include "frontend/crosshair.hpp"
#include "frontend/debugShapes.hpp"
#include "frontend/particleSystem.hpp"
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

    std::vector<PhysicsEngineReceiverData> physicsObjects;

    if (!currentSceneData.physicsObjects.empty()) {
        physicsObjects = Scene::SceneLoader::convertToPhysicsObjects(
          currentSceneData.physicsObjects);
    } else {
        // Fallback to hardcoded objects if no physics objects in scene
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
}

void Renderer::update()
{
    // Render static objects from scene
    renderStaticObjects();

    {
        bool isThisNewData = bridgeChannel->isMessageReady();
        auto message = bridgeChannel->receive();

        {
            // check for splash conditions and emit particles (only when
            // simulation is running)
            if (isThisNewData && currentSceneData.waterEnabled) {
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

            // update particles only when simulation is running
            // check if we received new data to determine if simulation is
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

        renderPhysicsObjects(message.getBuffer().physicsObjects);

        // should be second last thing drawn except bounding boxes and particles
        {
            skybox.setUniformsAndDraw(mainCamera);
        }

        if (DEBUGMODE) {
            // physics & light aabbs
            aabbs.clear();
            for (const auto& rigidBody : message.getBuffer().physicsObjects) {
                aabbs.emplace_back(rigidBody.computeAABB());
            }
            for (const auto& light : pointLights) {
                aabbs.emplace_back(light.computeAABB());
            }
            aabbVisualiser.draw(aabbs, mainCamera.getViewMatrix(),
                                mainCamera.getPerspectiveMatrix());
        }

        // should be last thing drawn except particles
        if (currentSceneData.waterEnabled) {
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

void Renderer::loadSceneData()
{
    if (!sceneFilePath.empty()) {
        try {
            currentSceneData = Scene::SceneLoader::loadFromFile(sceneFilePath);
        } catch (const std::exception& e) {
            throw IrrecoverableError{"Failed to load scene file '" +
                                     sceneFilePath.string() + "': " + e.what()};
        }
    } else {
        // Create default scene data with empty static objects and lights
        currentSceneData = Scene::SceneData{};
    }

    // Load static models
    staticModels.clear();
    staticObjects.clear();
    staticModels.reserve(currentSceneData.staticObjects.size());
    staticObjects.reserve(currentSceneData.staticObjects.size());

    for (const auto& staticObj : currentSceneData.staticObjects) {
        staticModels.emplace_back(
          std::make_unique<Model>(std::filesystem::path{staticObj.modelPath}));
        staticObjects.push_back(staticObj);
    }

    // Setup point lights
    pointLights.clear();
    pointLights.reserve(currentSceneData.pointLights.size());

    for (const auto& lightData : currentSceneData.pointLights) {
        linalg::aliases::float3 position{
          lightData.position[0], lightData.position[1], lightData.position[2]};
        linalg::aliases::float3 ambient{lightData.ambientColour[0],
                                        lightData.ambientColour[1],
                                        lightData.ambientColour[2]};
        linalg::aliases::float3 diffuse{lightData.diffuseColour[0],
                                        lightData.diffuseColour[1],
                                        lightData.diffuseColour[2]};
        linalg::aliases::float3 specular{lightData.specularColour[0],
                                         lightData.specularColour[1],
                                         lightData.specularColour[2]};

        pointLights.emplace_back(
          position, ambient, diffuse, specular, lightData.constantFalloff,
          lightData.linearFalloff, lightData.quadraticFalloff);
    }
}

void Renderer::renderStaticObjects()
{
    for (size_t i = 0; i < staticModels.size(); ++i) {
        const auto& staticObj = staticObjects[i];
        auto& model = staticModels[i];

        Shader* currentShader = nullptr;

        // Choose shader based on static object configuration
        switch (staticObj.shader) {
            case Scene::ShaderType::FLAT: currentShader = &flatShader; break;
            case Scene::ShaderType::LIGHT: currentShader = &shader; break;
            case Scene::ShaderType::SUN: currentShader = &sunShader; break;
        }

        if (currentShader != nullptr) {
            Shader::BindObject boundShader = currentShader->bind();
            mainCamera.setUniforms(boundShader);

            // Set up lights for light and sun shaders
            if (staticObj.shader == Scene::ShaderType::LIGHT) {
                setupPointLights(boundShader);
            }

            // Calculate transformation matrix
            linalg::aliases::float3 position{staticObj.position[0],
                                             staticObj.position[1],
                                             staticObj.position[2]};
            linalg::aliases::float3 scale{
              staticObj.scale[0], staticObj.scale[1], staticObj.scale[2]};

            // convert Euler angles (degrees) to radians and create quaternion
            float pitchRadians = staticObj.rotation[0] * (M_PI / 180.0F);
            float yawRadians = staticObj.rotation[1] * (M_PI / 180.0F);
            float rollRadians = staticObj.rotation[2] * (M_PI / 180.0F);

            Quaternion orientation = Quaternion::fromEulerAngles(
              rollRadians, pitchRadians, yawRadians);

            model->updateModelMatrix(position, orientation, scale);
            model->draw(boundShader);
        }
    }
}

void Renderer::setupPointLights(Shader::BindObject& shader)
{
    // Set up point lights for shaders that support them
    for (size_t i = 0; i < pointLights.size() && i < 4; ++i) {
        shader.setUniformInt("lightNum", static_cast<int>(pointLights.size()));
        pointLights[i].setUniforms(shader, static_cast<int>(i));
    }
}

const Scene::SceneData& Renderer::getSceneData() const
{
    return currentSceneData;
}

void Renderer::renderPhysicsObjects(const std::vector<RigidBodyData>& bodies)
{
    // Group physics objects by shader type for efficiency
    std::vector<const RigidBodyData*> flatObjects;
    std::vector<const RigidBodyData*> lightObjects;
    std::vector<const RigidBodyData*> sunObjects;

    size_t physicsObjIndex = 0;
    for (const auto& rigidBody : bodies) {
        if (!rigidBody.enabled) {
            continue;
        }

        Scene::ShaderType shaderType = Scene::ShaderType::SUN; // Default
        if (physicsObjIndex < currentSceneData.physicsObjects.size()) {
            shaderType =
              currentSceneData.physicsObjects[physicsObjIndex].shader;
        }

        switch (shaderType) {
            case Scene::ShaderType::FLAT:
                flatObjects.push_back(&rigidBody);
                break;
            case Scene::ShaderType::LIGHT:
                lightObjects.push_back(&rigidBody);
                break;
            case Scene::ShaderType::SUN:
                sunObjects.push_back(&rigidBody);
                break;
        }
        physicsObjIndex++;
    }

    // Render flat shader objects
    if (!flatObjects.empty()) {
        Shader::BindObject boundShader = flatShader.bind();
        mainCamera.setUniforms(boundShader);
        for (const auto* rigidBody : flatObjects) {
            Model& m = *(rigidBody->getModelPtr());
            m.updateModelMatrix(rigidBody->getWorldPosition(),
                                rigidBody->getOrientation(),
                                rigidBody->getScale());
            m.draw(boundShader);
        }
    }

    // Render light shader objects
    if (!lightObjects.empty()) {
        Shader::BindObject boundShader = shader.bind();
        mainCamera.setUniforms(boundShader);
        setupPointLights(boundShader);
        for (const auto* rigidBody : lightObjects) {
            Model& m = *(rigidBody->getModelPtr());
            m.updateModelMatrix(rigidBody->getWorldPosition(),
                                rigidBody->getOrientation(),
                                rigidBody->getScale());
            m.draw(boundShader);
        }
    }

    // Render sun shader objects
    if (!sunObjects.empty()) {
        Shader::BindObject boundShader = sunShader.bind();
        mainCamera.setUniforms(boundShader);
        for (const auto* rigidBody : sunObjects) {
            Model& m = *(rigidBody->getModelPtr());
            m.updateModelMatrix(rigidBody->getWorldPosition(),
                                rigidBody->getOrientation(),
                                rigidBody->getScale());
            m.draw(boundShader);
        }
    }
}
