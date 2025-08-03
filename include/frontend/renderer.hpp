#include "controller.hpp"
#include "frontend/aabbVisualiser.hpp"
#include "frontend/camera.hpp"
#include "frontend/particleSystem.hpp"
#include "frontend/skybox.hpp"
#include "model.hpp"
#include "physics/physicsEngine.hpp"
#include "pointLight.hpp"
#include "scene.hpp"
#include "sim/waterMesh.hpp"
#include "sim/waterSimulation.hpp"
#include "util/channel.hpp"
#include "util/logger.hpp"
#include "util/perf.hpp"
#include "util/singleton.hpp"
#include "window.hpp"

#include "GLFW/glfw3.h"
#include <filesystem>
#include <string>

struct BridgeChannelData;

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

    ParticleSystem particleSystem;
    ParticleRenderer particleRenderer;

    static inline std::vector<std::string> genLightUniformNames()
    {
        std::vector<std::string> names;

        std::vector<std::string> uniformsPerLight{
          "lights[{}].position",        "lights[{}].ambient",
          "lights[{}].diffuse",         "lights[{}].specular",
          "lights[{}].constantFalloff", "lights[{}].linearFalloff",
          "lights[{}].quadraticFalloff"};

        for (int i = 0; i < 4; ++i) {
            for (const auto& uni : uniformsPerLight) {
                std::string replaced = uni;
                auto pos = replaced.find("{}");
                if (pos != std::string::npos) {
                    replaced.replace(pos, 2, std::to_string(i));
                }
                names.push_back(replaced);
            }
        }

        names.emplace_back("lightNum");

        return names;
    };

    template <typename T>
    static auto concatenate(const std::vector<T>& lhs,
                            const std::vector<T>& rhs) -> std::vector<T>
    {
        auto result = lhs;
        std::copy(rhs.begin(), rhs.end(), std::back_inserter(result));
        return result;
    }

    std::vector<std::string> lightUniNames = genLightUniformNames();

    // TODO: Make it easier for user to change the shader and model without
    // having to come in here and poke about
    // Maybe can have a list of models and shaders, and some datastructure can
    // specify which model to draw with which shader with the camera or not etc.
    Shader shader{
      std::filesystem::path{"shaders/vertex/modelLightPerspTextureShader.glsl"},
      std::filesystem::path{"shaders/fragment/blinnPhongTextureShader.glsl"},
      // TODO: surely there's a better way than listing ALL of these sjsjsjs
      // TODO: oh god arrays...oh god...i REALLY need a better way for this
      concatenate(std::vector<std::string>{"projection", "view", "model",
                                           "material.diffuse",
                                           "material.specular", "material.Kd",
                                           "material.Ks", "material.Ns",
                                           "viewPos"},
                  lightUniNames)

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
      std::filesystem::path{"shaders/geometry/phongWaterSurface.glsl"},
      // std::filesystem::path{"shaders/fragment/simpleWaterSurface.glsl"},
      std::filesystem::path{"shaders/fragment/blinnPhongWaterSurface.glsl"},
      {"projection", "view", "model", "cameraPos", "skybox"}
    };

    PointLight light;

    // Model mainModel{
    //   std::filesystem::path{"assets/models/teapot-brick-big/teapot.obj"}};
    // Model teaPot2{
    //   std::filesystem::path{"assets/models/teapot-brick/teapot.obj"}};
    Model pool{std::filesystem::path{"assets/models/pool/pool.obj"}};

    Skybox skybox;

    AABBVisualiser aabbVisualiser;
    std::vector<AABB> aabbs = std::vector<AABB>(30);

    void update();

    bool DEBUGMODE = false;

    std::vector<std::unique_ptr<Model>> staticModels;
    std::vector<Scene::StaticObject> staticObjects;

    // Point lights (up to 4)
    std::vector<PointLight> pointLights;

    std::filesystem::path sceneFilePath;
    Scene::SceneData currentSceneData;

    std::vector<PhysicsEngineReceiverData> createDefaultScene();
    void renderStaticObjects();
    void renderPhysicsObjects(const std::vector<RigidBodyData>& bodies);
    void setupPointLights(Shader::BindObject& shader);

  public:
    Renderer(const Renderer&) = delete;
    Renderer(Renderer&&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    Renderer& operator=(Renderer&&) = delete;

    void init();

    void loadSceneData();

    [[nodiscard]] const Scene::SceneData& getSceneData() const;

    // TODO: Must restructure for the following
    // Minimize per-frame data transfers from CPU to GPU
    // Minimize number of state changes (binding framebuffers, textures,
    // shaders, buffers, etcetera)
    // Minimize number of individual draw calls per frame
    void loop();

    void attachBridgeChannel(Receiver<BridgeChannelData>*);
    void attachPhysicsEngineCommandsChannel(
      Sender<std::vector<PhysicsEngineReceiverData>>*);

    void toggleDebugMode();

    void setSceneFile(const std::filesystem::path& scenePath);
};
