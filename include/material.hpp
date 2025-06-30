#pragma once

#include "texture.hpp"
#include <linalg.h>
#include <string>

class Shader;

class Material
{
  public:
    enum class Type : uint8_t { MAT_LAMBERTIAN, MAT_METAL, MAT_GLASS };

  private:
    Type type = Type::MAT_LAMBERTIAN;

    Texture texture;

    std::string name;

    float eta = 1.0F;
    float glossiness = 1.0F;
    float specularExponent_Ns = 0.0F; // shininess

    linalg::aliases::float3 reflectivity_Ka{0.0F};
    linalg::aliases::float3 specularColour_Ks{0.0F};

    // default colour to know something is wrong
    linalg::aliases::float3 diffuseColour_Kd{0.9F, 0.0F, 0.0F};

  public:
    Material() = default;

    void setName(const std::string& n);
    void setType(Type t);
    void setAmbient(const linalg::aliases::float3& ka);
    void setDiffuse(const linalg::aliases::float3& kd);
    void setSpecular(const linalg::aliases::float3& ks);
    void setSpecularExponent(float ns);
    void setEta(float e);
    void setGlossiness(float g);
    void loadTexture(const std::filesystem::path& path);

    [[nodiscard]] bool isTextured() const;
    [[nodiscard]] const Texture& getTexture() const;
};
