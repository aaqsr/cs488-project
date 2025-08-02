#include "frontend/scene.hpp"
#include "frontend/model.hpp"
#include "physics/physicsEngine.hpp"
#include "util/error.hpp"

#include <linalg.h>

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace Scene
{

std::vector<SceneObject>
SceneLoader::loadFromFile(const std::filesystem::path& scenePath)
{
    if (!std::filesystem::exists(scenePath)) {
        throw IrrecoverableError{"Scene file does not exist: " +
                                 scenePath.string()};
    }

    std::ifstream file(scenePath);
    if (!file.is_open()) {
        throw IrrecoverableError{"Failed to open scene file: " +
                                 scenePath.string()};
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string jsonContent = buffer.str();

    json::JsonValue root;
    try {
        root = json::parse(jsonContent);
    } catch (const std::exception& e) {
        throw IrrecoverableError{"Failed to parse JSON in scene file: " +
                                 std::string(e.what())};
    }

    if (!root.isObject()) {
        throw IrrecoverableError{"Scene file root must be a JSON object"};
    }

    const auto& rootObj = root.asObject();
    auto objectsIt = rootObj.find("physicsObjects");
    if (objectsIt == rootObj.end()) {
        throw IrrecoverableError{"Scene file must contain an 'objects' array"};
    }

    if (!objectsIt->second.isArray()) {
        throw IrrecoverableError{"'objects' must be an array in scene file"};
    }

    const auto& objectsArray = objectsIt->second.asArray();
    std::vector<SceneObject> sceneObjects;
    sceneObjects.reserve(objectsArray.size());

    for (const auto& objJson : objectsArray) {
        if (!objJson.isObject()) {
            throw IrrecoverableError{
              "Each object in 'objects' array must be a JSON object"};
        }

        SceneObject obj = parseSceneObject(objJson);
        validateSceneObject(obj);
        sceneObjects.push_back(std::move(obj));
    }

    return sceneObjects;
}

SceneObject SceneLoader::parseSceneObject(const json::JsonValue& objJson)
{
    const auto& obj = objJson.asObject();
    SceneObject sceneObj;

    // required fields
    auto getRequiredField =
      [&obj](const std::string& fieldName) -> const json::JsonValue& {
        auto it = obj.find(fieldName);
        if (it == obj.end()) {
            throw IrrecoverableError{"Missing required field '" + fieldName +
                                     "' in scene object"};
        }
        return it->second;
    };

    // model path
    const auto& modelPathJson = getRequiredField("modelPath");
    if (!modelPathJson.isString()) {
        throw IrrecoverableError{"'modelPath' must be a string"};
    }
    sceneObj.modelPath = modelPathJson.asString();

    // scale (can be single number or array of 3)
    const auto& scaleJson = getRequiredField("scale");
    if (scaleJson.isNumber()) {
        float scaleVal = static_cast<float>(scaleJson.asNumber());
        sceneObj.scale[0] = sceneObj.scale[1] = sceneObj.scale[2] = scaleVal;
    } else if (scaleJson.isArray()) {
        const auto& scaleArray = scaleJson.asArray();
        if (scaleArray.size() != 3) {
            throw IrrecoverableError{
              "'scale' array must have exactly 3 elements"};
        }
        for (size_t i = 0; i < 3; ++i) {
            if (!scaleArray[i].isNumber()) {
                throw IrrecoverableError{"All scale values must be numbers"};
            }
            sceneObj.scale[i] = static_cast<float>(scaleArray[i].asNumber());
        }
    } else {
        throw IrrecoverableError{
          "'scale' must be a number or array of 3 numbers"};
    }

    // helper lambda to parse 3D vectors
    auto parse3DVector = [](const json::JsonValue& vectorJson,
                            const std::string& fieldName, float output[3]) {
        if (!vectorJson.isArray()) {
            throw IrrecoverableError{"'" + fieldName +
                                     "' must be an array of 3 numbers"};
        }
        const auto& array = vectorJson.asArray();
        if (array.size() != 3) {
            throw IrrecoverableError{"'" + fieldName +
                                     "' array must have exactly 3 elements"};
        }
        for (size_t i = 0; i < 3; ++i) {
            if (!array[i].isNumber()) {
                throw IrrecoverableError{"All '" + fieldName +
                                         "' values must be numbers"};
            }
            output[i] = static_cast<float>(array[i].asNumber());
        }
    };

    // initial position
    parse3DVector(getRequiredField("initPos"), "initPos", sceneObj.initPos);

    // initial velocity
    parse3DVector(getRequiredField("initVel"), "initVel", sceneObj.initVel);

    // initial angular velocity
    parse3DVector(getRequiredField("initAngVel"), "initAngVel",
                  sceneObj.initAngVel);

    // density
    const auto& densityJson = getRequiredField("density");
    if (!densityJson.isNumber()) {
        throw IrrecoverableError{"'density' must be a number"};
    }
    sceneObj.density = static_cast<float>(densityJson.asNumber());

    // and the optional name field
    auto nameIt = obj.find("name");
    if (nameIt != obj.end()) {
        if (!nameIt->second.isString()) {
            throw IrrecoverableError{"'name' must be a string"};
        }
        sceneObj.name = nameIt->second.asString();
    } else {
        sceneObj.name = "Unnamed Object";
    }

    return sceneObj;
}

void SceneLoader::validateSceneObject(const SceneObject& obj)
{
    if (!std::filesystem::exists(obj.modelPath)) {
        throw IrrecoverableError{"Model file does not exist: " + obj.modelPath};
    }

    for (float i : obj.scale) {
        if (i <= 0.0F) {
            throw IrrecoverableError{
              "Scale values must be positive for object: " + obj.name};
        }
    }

    if (obj.density <= 0.0F) {
        throw IrrecoverableError{"Density must be positive for object: " +
                                 obj.name};
    }
}

std::vector<PhysicsEngineReceiverData> SceneLoader::convertToPhysicsObjects(
  const std::vector<SceneObject>& sceneObjects)
{
    std::vector<PhysicsEngineReceiverData> physicsObjects;
    physicsObjects.reserve(sceneObjects.size());

    for (const auto& sceneObj : sceneObjects) {
        PhysicsEngineReceiverData physObj{
          .model =
            std::make_unique<Model>(std::filesystem::path{sceneObj.modelPath}
            ),
          .scale = linalg::aliases::float3{sceneObj.scale[0], sceneObj.scale[1],
                                                          sceneObj.scale[2]},
          .initPos =
            linalg::aliases::float3{sceneObj.initPos[0], sceneObj.initPos[1],
                                                          sceneObj.initPos[2]},
          .initVel =
            linalg::aliases::float3{sceneObj.initVel[0], sceneObj.initVel[1],
                                                          sceneObj.initVel[2]},
          .initAngVel = linalg::aliases::float3{sceneObj.initAngVel[0],
                                                          sceneObj.initAngVel[1],
                                                          sceneObj.initAngVel[2]},
          .density = sceneObj.density
        };

        physicsObjects.push_back(std::move(physObj));
    }

    return physicsObjects;
}

} // namespace Scene
