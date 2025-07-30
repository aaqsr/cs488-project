#include "controller.hpp"
#include "frontend/camera.hpp"
#include "frontend/skybox.hpp"
#include "model.hpp"
#include "physics/physicsEngine.hpp"
#include "pointLight.hpp"
#include "sim/waterMesh.hpp"
#include "sim/waterSimulation.hpp"
#include "util/channel.hpp"
#include "util/logger.hpp"
#include "util/perf.hpp"
#include "util/singleton.hpp"
#include "window.hpp"

#include "GLFW/glfw3.h"
#include <filesystem>

struct BridgeChannelData;
class RigidBody;

class Renderer : public Singleton<Renderer>
{
    friend class Singleton<Renderer>;
    friend class Window; // for the resize callback

    // DO NOT REORDER THESE
    Window& window = Window::GetInstance();
    Controller& controller = Controller::GetInstance();
    Camera mainCamera;

    double lastFrameTime = glfwGetTime();
    double currentFrameTime = 0.0;
    double deltaTime = 0.0;

    IterationsPerSecondCounter msPerFrame{"FPS", "frame"};

    Renderer();

    Receiver<BridgeChannelData>* bridgeChannel = nullptr;

    Sender<std::vector<PhysicsEngineReceiverData>>* physCmdChannel = nullptr;

    WaterMesh waterMesh;

    // TODO: Make it easier for user to change the shader and model without
    // having to come in here and poke about
    // Maybe can have a list of models and shaders, and some datastructure can
    // specify which model to draw with which shader with the camera or not etc.
    Shader shader{
      std::filesystem::path{"shaders/vertex/modelLightPerspTextureShader.glsl"},
      std::filesystem::path{"shaders/fragment/blinnPhongTextureShader.glsl"},
      // TODO: surely there's a better way than listing ALL of these sjsjsjs
      // TODO: oh god arrays...oh god...i REALLY need a better way for this
      {"projection", "view", "model", "material.diffuse", "material.specular",
                            "material.Kd", "material.Ks", "material.Ns", "viewPos",
                            "lights[0].position", "lights[0].ambient", "lights[0].diffuse",
                            "lights[0].specular", "lights[0].constantFalloff",
                            "lights[0].linearFalloff", "lights[0].quadraticFalloff"}
    };

    Shader flatShader{
      std::filesystem::path{"shaders/vertex/modelPerspTextureShader.glsl"},
      std::filesystem::path{"shaders/fragment/textureShader.glsl"},
      {"projection", "view", "model"}
    };

    // TODO: NEEDS A LOT OF WORK
    Shader sunShader{
      std::filesystem::path{"shaders/vertex/modelLightPerspTextureShader.glsl"},
      std::filesystem::path{"shaders/fragment/sunBlinnPhongTextureShader.glsl"},
      {"projection", "view", "model", "material.diffuse", "material.specular",
                            "material.Kd", "material.Ks", "material.Ns", "viewPos"}
    };

    Shader waterShader{
      std::filesystem::path{"shaders/vertex/waterSurface.glsl"},
      std::filesystem::path{"shaders/geometry/waterSurface.glsl"},
      std::filesystem::path{"shaders/fragment/simpleWaterSurface.glsl"},
      // std::filesystem::path{"shaders/fragment/reflectiveWaterSurface.glsl"},
      // std::filesystem::path{"shaders/fragment/pbrWaterSurface.glsl"},
      // std::filesystem::path{"shaders/fragment/opaqueWaterSurface.glsl"},
      // std::filesystem::path{"shaders/fragment/cartoonyReflectiveWaterSurface.glsl"},
      // std::filesystem::path{"shaders/fragment/fastReflectiveWaterSurface.glsl"},
      {
                            "projection", "view", "model",
                            // "cameraPos", "skybox"
      }
    };

    PointLight light;

    // Model mainModel{
    //   std::filesystem::path{"assets/models/teapot-brick-big/teapot.obj"}};
    // Model teaPot2{
    //   std::filesystem::path{"assets/models/teapot-brick/teapot.obj"}};
    Model pool{std::filesystem::path{"assets/models/pool/pool.obj"}};

    Skybox skybox;

    void update();

  public:
    Renderer(const Renderer&) = delete;
    Renderer(Renderer&&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    Renderer& operator=(Renderer&&) = delete;

    void init();
    // TODO: Must restructure for the following
    // Minimize per-frame data transfers from CPU to GPU
    // Minimize number of state changes (binding framebuffers, textures,
    // shaders, buffers, etcetera)
    // Minimize number of individual draw calls per frame
    void loop();

    void attachBridgeChannel(Receiver<BridgeChannelData>*);

    void attachPhysicsEngineCommandsChannel(
      Sender<std::vector<PhysicsEngineReceiverData>>*);
};
