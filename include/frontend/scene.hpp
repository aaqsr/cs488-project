#pragma once

#include "bridgeChannelData.hpp"
#include "util/json.hpp"

#include <filesystem>
#include <memory>
#include <vector>

struct PhysicsEngineReceiverData;

namespace Scene
{

enum class ShaderType : uint8_t { FLAT, LIGHT, SUN };

struct PointLightData
{
    float position[3];
    float ambientColour[3];
    float diffuseColour[3];
    float specularColour[3];
    float constantFalloff;
    float linearFalloff;
    float quadraticFalloff;
};

struct PhysicsObject
{
    std::string modelPath;
    float scale[3];
    float initPos[3];
    float initVel[3];
    float initAngVel[3];
    float density;
    ShaderType shader;
    std::string name; // optional, for debugging/identification
};

struct StaticObject
{
    std::string modelPath;
    float scale[3];
    float position[3];
    float rotation[3]; // euler angles in degrees
    ShaderType shader;
    std::string name; // optional, for debugging/identification
};

struct SceneData
{
    std::vector<PhysicsObject> physicsObjects;
    std::vector<StaticObject> staticObjects;
    std::vector<PointLightData> pointLights; // max 4 lights
    bool waterEnabled = true;
    float waterInitialHumpSize = 0.4F;
};

class SceneLoader
{
  public:
    static SceneData loadFromFile(const std::filesystem::path& scenePath);
    static std::vector<PhysicsEngineReceiverData>
    convertToPhysicsObjects(const std::vector<PhysicsObject>& physicsObjects);

  private:
    static PhysicsObject parsePhysicsObject(const json::JsonValue& objJson);
    static StaticObject parseStaticObject(const json::JsonValue& objJson);
    static PointLightData parsePointLight(const json::JsonValue& lightJson);
    static ShaderType parseShaderType(const std::string& shaderStr);
    static void validatePhysicsObject(const PhysicsObject& obj);
    static void validateStaticObject(const StaticObject& obj);
    static void validatePointLight(const PointLightData& light);
};

} // namespace Scene
