#include "frontend/model.hpp"
#include "frontend/window.hpp"
#include "util/logger.hpp"
#include "util/singleton.hpp"
#include <filesystem>

class Crosshair : public Singleton<Crosshair>
{
    friend class Singleton<Crosshair>;

    constexpr static float size = 0.5F;

    static Model createCrosshairModel()
    {
        auto material = std::make_shared<Material>();
        material->loadDiffuseMap(
          std::filesystem::path{"assets/textures/crosshair.png"});
        material->setName("CrosshairMaterial");
        material->setType(Material::Type::MAT_LAMBERTIAN);

        const float unitHeight = 1.0F;
        const float unitWidth = 1.0F;

        std::vector<Vertex> vertices = {
          // Bottom-left
          {.position = {-unitWidth / 2.0F, -unitHeight / 2.0F, 0.0F},
           .normal = {0.0F, 0.0F, 0.0F},
           .texCoords = {1.0F, 0.0F}},
          // Bottom-right
          { .position = {unitWidth / 2.0F, -unitHeight / 2.0F, 0.0F},
           .normal = {0.0F, 0.0F, 0.0F},
           .texCoords = {1.0F, 1.0F}},
          // Top-right
          {  .position = {unitWidth / 2.0F, unitHeight / 2.0F, 0.0F},
           .normal = {0.0F, 0.0F, 0.0F},
           .texCoords = {0.0F, 1.0F}},
          // Top-left
          { .position = {-unitWidth / 2.0F, unitHeight / 2.0F, 0.0F},
           .normal = {0.0F, 0.0F, 0.0F},
           .texCoords = {0.0F, 0.0F}}
        };

        // Indices for two triangles forming a square
        std::vector<unsigned int> indices = {
          0, 1, 2, // First triangle (bottom-left, bottom-right, top-right)
          2, 3, 0  // Second triangle (top-right, top-left, bottom-left)
        };

        std::vector<Mesh> meshes;
        meshes.emplace_back(std::move(vertices), std::move(indices), material);

        return Model{std::move(meshes),
                     std::unordered_map<std::string, std::shared_ptr<Material>>{
                       {"", material}}};
    }

    Shader noPerspectiveTexShader{
      std::filesystem::path{"shaders/vertex/crosshairShader.glsl"},
      std::filesystem::path{"shaders/fragment/textureShader.glsl"},
      std::vector<std::string>{"material.diffuse", "model"}
    };

    Model crosshairModel{createCrosshairModel()};

    Crosshair()
    {
        updateSize(Window::width, Window::height);
    }

  public:
    void updateSize(int windowWidth, int windowHeight)
    {
        const float aspectRatio =
          static_cast<float>(windowWidth) / static_cast<float>(windowHeight);
        const float width = size * (1.0F / aspectRatio);
        const float height = size;

        crosshairModel.updateModelMatrix({}, {}, {width, height, 0.0F});
    }

    void draw()
    {

        // store depth testing
        GLboolean currentDepthMask;
        glGetBooleanv(GL_DEPTH_WRITEMASK, &currentDepthMask);

        // disable depth testing
        glDepthMask(GL_FALSE);

        // should probably store and restore the blend state too but ehhh
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        auto shader = noPerspectiveTexShader.bind();
        crosshairModel.draw(shader);

        // Restore states
        glDisable(GL_BLEND);
        glDepthMask(currentDepthMask);
    }
};
