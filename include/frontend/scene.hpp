#pragma once

#include "bridgeChannelData.hpp"
#include "util/json.hpp"

#include <filesystem>
#include <memory>
#include <vector>

struct PhysicsEngineReceiverData;

namespace Scene
{

struct SceneObject
{
    std::string modelPath;
    float scale[3];
    float initPos[3];
    float initVel[3];
    float initAngVel[3];
    float density;
    std::string name; // optional, for debugging/identification
};

class SceneLoader
{
  public:
    static std::vector<SceneObject>
    loadFromFile(const std::filesystem::path& scenePath);
    static std::vector<PhysicsEngineReceiverData>
    convertToPhysicsObjects(const std::vector<SceneObject>& sceneObjects);

  private:
    static SceneObject parseSceneObject(const json::JsonValue& objJson);
    static void validateSceneObject(const SceneObject& obj);
};

} // namespace Scene
