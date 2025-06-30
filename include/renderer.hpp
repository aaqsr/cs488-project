#pragma once

#include "controller.hpp"
#include "logger.hpp"
#include "model.hpp"
#include "singleton.hpp"
#include "window.hpp"

#include <GLFW/glfw3.h>

class Renderer : public Singleton<Renderer>
{
    friend class Singleton<Renderer>;

    // DO NOT REORDER THESE
    Window& window = Window::GetInstance();
    Controller& controller = Controller::GetInstance();
    Camera mainCamera;
    double lastFrameTime = glfwGetTime();
    double currentFrameTime = 0.0;
    double deltaTime = 0.0;

    class MsPerFrameCounter
    {
        uint32_t numFrames = 0;
        double timeOfLastPrint;

      public:
        void tick(double currentFrameTime)
        {
            numFrames++;
            if (currentFrameTime - timeOfLastPrint >= 1.0F) {
                Logger::GetInstance().log(
                  std::format("{:<6}FPS\t{:<9.5f}ms/frame", numFrames,
                              1000.0 / double(numFrames)));
                numFrames = 0;
                timeOfLastPrint = currentFrameTime;
            }
        }
        MsPerFrameCounter(double timeOfLastPrint)
          : timeOfLastPrint{timeOfLastPrint} { };
    } msPerFrame{lastFrameTime};

    Renderer();

    // TODO: Make it easier for user to change the shader and model without
    // having to come in here and poke about
    // Maybe can have a list of models and shaders, and some datastructure can
    // specify which model to draw with which shader with the camera or not etc.
    Shader texShader{
      std::filesystem::path{"shaders/vertex/perspTextureShader.glsl"},
      std::filesystem::path{"shaders/fragment/textureShader.glsl"},
      {"projection", "view"}
    };

    Model teapot{
      std::filesystem::path{"assets/models/teapot-ajrak/teapot.obj"}};

    void drawLoop();

  public:
    Renderer(const Renderer&) = delete;
    Renderer(Renderer&&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    Renderer& operator=(Renderer&&) = delete;

    // TODO: Must restructure for the following
    // Minimize per-frame data transfers from CPU to GPU
    // Minimize number of state changes (binding framebuffers, textures,
    // shaders, buffers, etcetera)
    // Minimize number of individual draw calls per frame
    void loop();
};
