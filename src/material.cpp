#include "material.hpp"

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

void Material::loadTexture(const std::filesystem::path& path)
{
    texture = Texture(path);
}

bool Material::isTextured() const
{
    return texture.isValid();
}

const Texture& Material::getTexture() const
{
    return texture;
}
