#pragma once

#include "linalg.h"
#include "texture.hpp"
#include <string>

class Shader;

class Material
{
  public:
    enum class Type : uint8_t { MAT_LAMBERTIAN, MAT_METAL, MAT_GLASS };

  private:
    // unused right now. Here if we want to do BRDF stuff later
    Type type = Type::MAT_LAMBERTIAN;

    // Texture if there is no underlying texture map
    inline const static std::array<unsigned char, 3> defaultTexData{255, 255,
                                                                    255};
    // we defer actual initialisation of this if it gets used later
    inline static Texture defaultTexture{};

    Texture diffuseMap;
    Texture specularMap;

    std::string name;

    float eta = 1.0F;
    float glossiness = 1.0F;
    float specularExponent_Ns = 0.0F; // shininess

    linalg::aliases::float3 reflectivity_Ka{0.0F};

    linalg::aliases::float3 specularColour_Ks{0.0F};

    // default colour to know something is wrong
    linalg::aliases::float3 diffuseColour_Kd{0.9F, 0.0F, 0.0F};

    static const Texture& getDefaultTexture();

  public:
    Material() = default;

    // DOES NOT BIND SHADER BEFORE SETTING UNIFORMS. MEANT TO BE USED IN DRAWING
    // LOOP AFTER SHADER IS SET.
    // TODO: Restructure code to be better somehow?? Maybe an RAII object that
    // remembers to unbind?
    void setUniformsAndBind(Shader::BindObject& shader);
    void unbind();

    void setName(const std::string& n);
    void setType(Type t);
    void setAmbient(const linalg::aliases::float3& ka);
    void setDiffuse(const linalg::aliases::float3& kd);
    void setSpecular(const linalg::aliases::float3& ks);
    void setSpecularExponent(float ns);
    void setEta(float e);
    void setGlossiness(float g);
    void loadDiffuseMap(const std::filesystem::path& path);
    void loadSpecularMap(const std::filesystem::path& path);

    [[nodiscard]] float getNs() const;
    [[nodiscard]] const linalg::aliases::float3& getKd() const;
    [[nodiscard]] const linalg::aliases::float3& getKs() const;
    [[nodiscard]] const Texture& getDiffuseMap() const;
    [[nodiscard]] const Texture& getSpecularMap() const;
};
