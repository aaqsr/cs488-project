#include "frontend/material.hpp"
#include "util/logger.hpp"
#include <string>

void Material::setName(const std::string& n)
{
    name = n;
}
void Material::setType(Type t)
{
    type = t;
}
void Material::setAmbient(const linalg::aliases::float3& ka)
{
    reflectivity_Ka = ka;
}
void Material::setDiffuse(const linalg::aliases::float3& kd)
{
    diffuseColour_Kd = kd;
}
void Material::setSpecular(const linalg::aliases::float3& ks)
{
    specularColour_Ks = ks;
}
void Material::setSpecularExponent(float ns)
{
    specularExponent_Ns = ns;
}
void Material::setEta(float e)
{
    eta = e;
}
void Material::setGlossiness(float g)
{
    glossiness = g;
}

void Material::loadDiffuseMap(const std::filesystem::path& path)
{
    diffuseMap = Texture(path);
}

void Material::loadSpecularMap(const std::filesystem::path& path)
{
    specularMap = Texture(path);
}

float Material::getNs() const
{
    return specularExponent_Ns;
}

const linalg::aliases::float3& Material::getKd() const
{
    return diffuseColour_Kd;
}

const linalg::aliases::float3& Material::getKs() const
{
    return specularColour_Ks;
}

const Texture& Material::getDefaultTexture()
{
    if (!Material::defaultTexture.isValid()) {
        Material::defaultTexture = {1, 1, defaultTexData.data()};
    }

    return Material::defaultTexture;
}

const Texture& Material::getDiffuseMap() const
{
    if (!diffuseMap.isValid()) {
        return getDefaultTexture();
    }

    return diffuseMap;
}

const Texture& Material::getSpecularMap() const
{
    if (!specularMap.isValid()) {
        return getDefaultTexture();
    }

    return specularMap;
}

void Material::setUniformsAndBind(Shader::BindObject& shader)
{
    shader.setUniform("material.Kd", getKd());
    shader.setUniform("material.Ks", getKs());
    shader.setUniform("material.Ns", getNs());

    shader.setUniformInt("material.isDiffuseTextured", diffuseMap.isValid());
    shader.setUniformInt("material.isSpecularTextured", specularMap.isValid());

    getDiffuseMap().bind(shader, "material.diffuse", 0);
    getSpecularMap().bind(shader, "material.specular", 1);
}

void Material::unbind()
{
    getDiffuseMap().unbind(0);
    getSpecularMap().unbind(1);
}
