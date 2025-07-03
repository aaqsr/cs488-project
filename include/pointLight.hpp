#pragma once

#include "shader.hpp"
#include <linalg.h>
#include <string>

// Does not use PBR (irradiance, wattage, etc.) because Phong lighting was
// easier and computationally cheaper to implement
// Also point light has no model yet...might wish to chage that to a small cube
// or something...
class PointLight
{
    // linalg::aliases::float3 worldPos{0.0F, 0.4F, 0.0F};
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

    // USER MUST BIND SHADER
    // TODO: there must be consistency... The convention could be that caller
    // always binds shader...
    void setUniforms(Shader::BindObject& shader, int lightNum);

    [[nodiscard]] const linalg::aliases::float3& getPos() const
    {
        return worldPos;
    }
};

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
