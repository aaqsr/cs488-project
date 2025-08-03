#pragma once

#include "linalg.h"
#include "physics/AABB.hpp"
#include "shader.hpp"
#include <string>

// Does not use PBR (irradiance, wattage, etc.) because Phong lighting was
// easier and computationally cheaper to implement
// Also point light has no model yet...might wish to change that to a small cube
// or something...
class PointLight
{
    linalg::aliases::float3 worldPos{0.0F, 0.2F, 1.0F};

    linalg::aliases::float3 ambientColour{0.1F, 0.1F, 0.1F};
    linalg::aliases::float3 diffuseColour{0.5F, 0.5F, 0.5F};
    linalg::aliases::float3 specularColour{0.5F, 0.5F, 0.5F};

    // Good source for these values: (we pick range 13 as example)
    // https://wiki.ogre3d.org/tiki-index.php?page=-Point+Light+Attenuation
    float constantFalloff = 1.0F;
    float linearFalloff = 0.35F;
    float quadraticFalloff = 0.44F;

  public:
    PointLight() = default;

    // Constructor for configurable point light
    PointLight(const linalg::aliases::float3& position,
               const linalg::aliases::float3& ambient,
               const linalg::aliases::float3& diffuse,
               const linalg::aliases::float3& specular, float constant,
               float linear, float quadratic);

    void setUniforms(Shader::BindObject& shader, int lightNum);

    [[nodiscard]] AABB computeAABB() const
    {
        return AABB{worldPos - linalg::aliases::float3(0.1F),
                    worldPos + linalg::aliases::float3(0.1)};
    }

    [[nodiscard]] const linalg::aliases::float3& getPos() const
    {
        return worldPos;
    }

    // Setters for runtime modification
    void setPosition(const linalg::aliases::float3& position)
    {
        worldPos = position;
    }
    void setAmbientColour(const linalg::aliases::float3& colour)
    {
        ambientColour = colour;
    }
    void setDiffuseColour(const linalg::aliases::float3& colour)
    {
        diffuseColour = colour;
    }
    void setSpecularColour(const linalg::aliases::float3& colour)
    {
        specularColour = colour;
    }
    void setFalloff(float constant, float linear, float quadratic)
    {
        constantFalloff = constant;
        linearFalloff = linear;
        quadraticFalloff = quadratic;
    }
};

inline PointLight::PointLight(const linalg::aliases::float3& position,
                              const linalg::aliases::float3& ambient,
                              const linalg::aliases::float3& diffuse,
                              const linalg::aliases::float3& specular,
                              float constant, float linear, float quadratic)
  : worldPos(position), ambientColour(ambient), diffuseColour(diffuse),
    specularColour(specular), constantFalloff(constant), linearFalloff(linear),
    quadraticFalloff(quadratic)
{
}

inline void PointLight::setUniforms(Shader::BindObject& shader, int lightNum)
{
    std::string varName{"lights[" + std::to_string(lightNum) + "]"};

    shader.setUniform(varName + ".position", worldPos);

    shader.setUniform(varName + ".ambient", ambientColour);
    shader.setUniform(varName + ".diffuse", diffuseColour);
    shader.setUniform(varName + ".specular", specularColour);

    shader.setUniform(varName + ".constantFalloff", constantFalloff);
    shader.setUniform(varName + ".linearFalloff", linearFalloff);
    shader.setUniform(varName + ".quadraticFalloff", quadraticFalloff);
}
