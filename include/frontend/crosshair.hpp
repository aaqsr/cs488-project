#include "frontend/model.hpp"
#include "frontend/shader.hpp"
#include "frontend/window.hpp"
#include "util/logger.hpp"
#include "util/singleton.hpp"

#include <cmath>
#include <linalg.h>

#include <filesystem>
#include <vector>

class HUDElement
{

    static Model createSquare(const linalg::aliases::float2& pos,
                              const std::filesystem::path& texture)
    {
        auto material = std::make_shared<Material>();
        material->loadDiffuseMap(texture);
        material->setName("CrosshairMaterial");
        material->setType(Material::Type::MAT_LAMBERTIAN);

        std::vector<Vertex> vertices = {
          // Bottom-left (centered at pos)
          {.position = {-0.5F, -0.5F, 0.0F},
           .normal = {0.0F, 0.0F, 0.0F},
           .texCoords = {0.0F, 0.0F}},
          // Bottom-right (centered at pos)
          { .position = {0.5F, -0.5F, 0.0F},
           .normal = {0.0F, 0.0F, 0.0F},
           .texCoords = {1.0F, 0.0F}},
          // Top-right (centered at pos)
          {  .position = {0.5F, 0.5F, 0.0F},
           .normal = {0.0F, 0.0F, 0.0F},
           .texCoords = {1.0F, 1.0F}},
          // Top-left (centered at pos)
          { .position = {-0.5F, 0.5F, 0.0F},
           .normal = {0.0F, 0.0F, 0.0F},
           .texCoords = {0.0F, 1.0F}}
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

    Model model;

    linalg::aliases::float2 position;
    float size;

  public:
    bool enabled = true;

    HUDElement(const linalg::aliases::float2& pos, float elementSize,
               const std::filesystem::path& texture, bool enabled = true)
      : model(createSquare(pos, texture)), position(pos), size(elementSize),
        enabled(enabled)
    {
    }

    virtual ~HUDElement() = default;

    void scale(float aspectRatio)
    {
        const float width = size * (1.0F / aspectRatio);
        const float height = size;
        model.updateModelMatrix({position.x, position.y, 0.0F}, {},
                                {width, height, 0.0F});
    }

    void draw(Shader::BindObject& shader)
    {
        // store depth testing
        GLboolean currentDepthMask;
        glGetBooleanv(GL_DEPTH_WRITEMASK, &currentDepthMask);

        // disable depth testing
        glDepthMask(GL_FALSE);

        // should probably store and restore the blend state too but ehhh
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        model.draw(shader);

        // Restore states
        glDisable(GL_BLEND);
        glDepthMask(currentDepthMask);
    }

    void toggle()
    {
        enabled = !enabled;
    }
};

class HUD : public Singleton<HUD>
{
    friend class Singleton<HUD>;

    Shader noPerspectiveTexShader{
      std::filesystem::path{"shaders/vertex/hudShader.glsl"},
      std::filesystem::path{"shaders/fragment/hudShader.glsl"},
      std::vector<std::string>{"material.diffuse", "model"}
    };

    HUDElement crosshair{
      {0.0F, 0.0F},
      0.1F,
      std::filesystem::path{"assets/textures/crosshair.png"},
      false
    };

    HUDElement pauseButton{
      {0.8F, 0.8F},
      0.1F,
      std::filesystem::path{"assets/textures/pauseButton.png"}
    };

    HUDElement playButton{
      {0.8F, 0.8F},
      0.1F,
      std::filesystem::path{"assets/textures/playButton.png"},
      false
    };

    std::vector<HUDElement*> hudElements{&crosshair, &pauseButton, &playButton};

    HUD()
    {
        updateSize(Window::width, Window::height);
    }

  public:
    void draw()
    {
        auto shader = noPerspectiveTexShader.bind();
        for (auto* e : hudElements) {
            if (e->enabled) {
                e->draw(shader);
            }
        }
    }

    void updateSize(int windowWidth, int windowHeight)
    {
        const float aspectRatio =
          static_cast<float>(windowWidth) / static_cast<float>(windowHeight);
        for (auto* e : hudElements) {
            e->scale(aspectRatio);
        }
    }

    void togglePause()
    {
        pauseButton.toggle();
        playButton.toggle();
        crosshair.toggle();
    }
};
