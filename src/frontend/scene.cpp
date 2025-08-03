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

SceneData SceneLoader::loadFromFile(const std::filesystem::path& scenePath)
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
    SceneData sceneData;

    // Parse physics objects (optional)
    auto physicsObjectsIt = rootObj.find("physicsObjects");
    if (physicsObjectsIt != rootObj.end()) {
        if (!physicsObjectsIt->second.isArray()) {
            throw IrrecoverableError{
              "'physicsObjects' must be an array in scene file"};
        }

        const auto& physicsArray = physicsObjectsIt->second.asArray();
        sceneData.physicsObjects.reserve(physicsArray.size());

        for (const auto& objJson : physicsArray) {
            if (!objJson.isObject()) {
                throw IrrecoverableError{"Each object in 'physicsObjects' "
                                         "array must be a JSON object"};
            }

            PhysicsObject obj = parsePhysicsObject(objJson);
            validatePhysicsObject(obj);
            sceneData.physicsObjects.push_back(std::move(obj));
        }
    }

    // Parse static objects (optional)
    auto staticObjectsIt = rootObj.find("staticObjects");
    if (staticObjectsIt != rootObj.end()) {
        if (!staticObjectsIt->second.isArray()) {
            throw IrrecoverableError{
              "'staticObjects' must be an array in scene file"};
        }

        const auto& staticArray = staticObjectsIt->second.asArray();
        sceneData.staticObjects.reserve(staticArray.size());

        for (const auto& objJson : staticArray) {
            if (!objJson.isObject()) {
                throw IrrecoverableError{
                  "Each object in 'staticObjects' array must be a JSON object"};
            }

            StaticObject obj = parseStaticObject(objJson);
            validateStaticObject(obj);
            sceneData.staticObjects.push_back(std::move(obj));
        }
    }

    // Parse point lights (optional, max 4)
    auto pointLightsIt = rootObj.find("pointLights");
    if (pointLightsIt != rootObj.end()) {
        if (!pointLightsIt->second.isArray()) {
            throw IrrecoverableError{
              "'pointLights' must be an array in scene file"};
        }

        const auto& lightsArray = pointLightsIt->second.asArray();
        if (lightsArray.size() > 4) {
            throw IrrecoverableError{"Maximum of 4 point lights are supported"};
        }

        sceneData.pointLights.reserve(lightsArray.size());

        for (const auto& lightJson : lightsArray) {
            if (!lightJson.isObject()) {
                throw IrrecoverableError{
                  "Each object in 'pointLights' array must be a JSON object"};
            }

            PointLightData light = parsePointLight(lightJson);
            validatePointLight(light);
            sceneData.pointLights.push_back(std::move(light));
        }
    }

    // Parse water configuration (optional)
    auto waterEnabledIt = rootObj.find("waterEnabled");
    if (waterEnabledIt != rootObj.end()) {
        if (!waterEnabledIt->second.isBool()) {
            throw IrrecoverableError{"'waterEnabled' must be a boolean"};
        }
        sceneData.waterEnabled = waterEnabledIt->second.asBool();
    } else {
        sceneData.waterEnabled = true;
    }

    auto waterHumpSizeIt = rootObj.find("waterInitialHumpSize");
    if (waterHumpSizeIt != rootObj.end()) {
        if (!waterHumpSizeIt->second.isNumber()) {
            throw IrrecoverableError{"'waterInitialHumpSize' must be a number"};
        }
        sceneData.waterInitialHumpSize =
          static_cast<float>(waterHumpSizeIt->second.asNumber());
    } else {
        sceneData.waterInitialHumpSize = 0.4F;
    }

    return sceneData;
}

PhysicsObject SceneLoader::parsePhysicsObject(const json::JsonValue& objJson)
{
    const auto& obj = objJson.asObject();
    PhysicsObject physicsObj;

    // required fields
    auto getRequiredField =
      [&obj](const std::string& fieldName) -> const json::JsonValue& {
        auto it = obj.find(fieldName);
        if (it == obj.end()) {
            throw IrrecoverableError{"Missing required field '" + fieldName +
                                     "' in physics object"};
        }
        return it->second;
    };

    // model path
    const auto& modelPathJson = getRequiredField("modelPath");
    if (!modelPathJson.isString()) {
        throw IrrecoverableError{"'modelPath' must be a string"};
    }
    physicsObj.modelPath = modelPathJson.asString();

    // scale (can be single number or array of 3)
    const auto& scaleJson = getRequiredField("scale");
    if (scaleJson.isNumber()) {
        float scaleVal = static_cast<float>(scaleJson.asNumber());
        physicsObj.scale[0] = physicsObj.scale[1] = physicsObj.scale[2] =
          scaleVal;
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
            physicsObj.scale[i] = static_cast<float>(scaleArray[i].asNumber());
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
    parse3DVector(getRequiredField("initPos"), "initPos", physicsObj.initPos);

    // initial velocity
    parse3DVector(getRequiredField("initVel"), "initVel", physicsObj.initVel);

    // initial angular velocity
    parse3DVector(getRequiredField("initAngVel"), "initAngVel",
                  physicsObj.initAngVel);

    // density
    const auto& densityJson = getRequiredField("density");
    if (!densityJson.isNumber()) {
        throw IrrecoverableError{"'density' must be a number"};
    }
    physicsObj.density = static_cast<float>(densityJson.asNumber());

    // shader type (optional, defaults to flat)
    auto shaderIt = obj.find("shader");
    if (shaderIt != obj.end()) {
        if (!shaderIt->second.isString()) {
            throw IrrecoverableError{"'shader' must be a string"};
        }
        physicsObj.shader = parseShaderType(shaderIt->second.asString());
    } else {
        physicsObj.shader = ShaderType::FLAT;
    }

    // optional name field
    auto nameIt = obj.find("name");
    if (nameIt != obj.end()) {
        if (!nameIt->second.isString()) {
            throw IrrecoverableError{"'name' must be a string"};
        }
        physicsObj.name = nameIt->second.asString();
    } else {
        physicsObj.name = "Unnamed Physics Object";
    }

    return physicsObj;
}

StaticObject SceneLoader::parseStaticObject(const json::JsonValue& objJson)
{
    const auto& obj = objJson.asObject();
    StaticObject staticObj;

    // required fields
    auto getRequiredField =
      [&obj](const std::string& fieldName) -> const json::JsonValue& {
        auto it = obj.find(fieldName);
        if (it == obj.end()) {
            throw IrrecoverableError{"Missing required field '" + fieldName +
                                     "' in static object"};
        }
        return it->second;
    };

    // model path
    const auto& modelPathJson = getRequiredField("modelPath");
    if (!modelPathJson.isString()) {
        throw IrrecoverableError{"'modelPath' must be a string"};
    }
    staticObj.modelPath = modelPathJson.asString();

    // scale (can be single number or array of 3)
    const auto& scaleJson = getRequiredField("scale");
    if (scaleJson.isNumber()) {
        float scaleVal = static_cast<float>(scaleJson.asNumber());
        staticObj.scale[0] = staticObj.scale[1] = staticObj.scale[2] = scaleVal;
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
            staticObj.scale[i] = static_cast<float>(scaleArray[i].asNumber());
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

    // position
    parse3DVector(getRequiredField("position"), "position", staticObj.position);

    // rotation (optional, defaults to 0,0,0)
    auto rotationIt = obj.find("rotation");
    if (rotationIt != obj.end()) {
        parse3DVector(rotationIt->second, "rotation", staticObj.rotation);
    } else {
        staticObj.rotation[0] = staticObj.rotation[1] = staticObj.rotation[2] =
          0.0F;
    }

    // shader type
    const auto& shaderJson = getRequiredField("shader");
    if (!shaderJson.isString()) {
        throw IrrecoverableError{"'shader' must be a string"};
    }
    staticObj.shader = parseShaderType(shaderJson.asString());

    // optional name field
    auto nameIt = obj.find("name");
    if (nameIt != obj.end()) {
        if (!nameIt->second.isString()) {
            throw IrrecoverableError{"'name' must be a string"};
        }
        staticObj.name = nameIt->second.asString();
    } else {
        staticObj.name = "Unnamed Static Object";
    }

    return staticObj;
}

PointLightData SceneLoader::parsePointLight(const json::JsonValue& lightJson)
{
    const auto& light = lightJson.asObject();
    PointLightData lightData;

    // required fields
    auto getRequiredField =
      [&light](const std::string& fieldName) -> const json::JsonValue& {
        auto it = light.find(fieldName);
        if (it == light.end()) {
            throw IrrecoverableError{"Missing required field '" + fieldName +
                                     "' in point light"};
        }
        return it->second;
    };

    // helper lambda to parse 3D vectors with default values
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

    // position (required)
    parse3DVector(getRequiredField("position"), "position", lightData.position);

    // ambient colour (optional, defaults to 0.1, 0.1, 0.1)
    auto ambientIt = light.find("ambientColour");
    if (ambientIt != light.end()) {
        parse3DVector(ambientIt->second, "ambientColour",
                      lightData.ambientColour);
    } else {
        lightData.ambientColour[0] = lightData.ambientColour[1] =
          lightData.ambientColour[2] = 0.1F;
    }

    // diffuse colour (optional, defaults to 0.5, 0.5, 0.5)
    auto diffuseIt = light.find("diffuseColour");
    if (diffuseIt != light.end()) {
        parse3DVector(diffuseIt->second, "diffuseColour",
                      lightData.diffuseColour);
    } else {
        lightData.diffuseColour[0] = lightData.diffuseColour[1] =
          lightData.diffuseColour[2] = 0.5F;
    }

    // specular colour (optional, defaults to 0.5, 0.5, 0.5)
    auto specularIt = light.find("specularColour");
    if (specularIt != light.end()) {
        parse3DVector(specularIt->second, "specularColour",
                      lightData.specularColour);
    } else {
        lightData.specularColour[0] = lightData.specularColour[1] =
          lightData.specularColour[2] = 0.5F;
    }

    // falloff parameters (optional, with sensible defaults)
    auto constantIt = light.find("constantFalloff");
    if (constantIt != light.end()) {
        if (!constantIt->second.isNumber()) {
            throw IrrecoverableError{"'constantFalloff' must be a number"};
        }
        lightData.constantFalloff =
          static_cast<float>(constantIt->second.asNumber());
    } else {
        lightData.constantFalloff = 1.0F;
    }

    auto linearIt = light.find("linearFalloff");
    if (linearIt != light.end()) {
        if (!linearIt->second.isNumber()) {
            throw IrrecoverableError{"'linearFalloff' must be a number"};
        }
        lightData.linearFalloff =
          static_cast<float>(linearIt->second.asNumber());
    } else {
        lightData.linearFalloff = 0.35F;
    }

    auto quadraticIt = light.find("quadraticFalloff");
    if (quadraticIt != light.end()) {
        if (!quadraticIt->second.isNumber()) {
            throw IrrecoverableError{"'quadraticFalloff' must be a number"};
        }
        lightData.quadraticFalloff =
          static_cast<float>(quadraticIt->second.asNumber());
    } else {
        lightData.quadraticFalloff = 0.44F;
    }

    return lightData;
}

ShaderType SceneLoader::parseShaderType(const std::string& shaderStr)
{
    if (shaderStr == "flat") {
        return ShaderType::FLAT;
    } else if (shaderStr == "light") {
        return ShaderType::LIGHT;
    } else if (shaderStr == "sun") {
        return ShaderType::SUN;
    } else {
        throw IrrecoverableError{"Unknown shader type '" + shaderStr +
                                 "'. Valid types are: 'flat', 'light', 'sun'"};
    }
}

void SceneLoader::validatePhysicsObject(const PhysicsObject& obj)
{
    if (!std::filesystem::exists(obj.modelPath)) {
        throw IrrecoverableError{"Model file does not exist: " + obj.modelPath};
    }

    for (float i : obj.scale) {
        if (i <= 0.0F) {
            throw IrrecoverableError{
              "Scale values must be positive for physics object: " + obj.name};
        }
    }

    if (obj.density <= 0.0F) {
        throw IrrecoverableError{
          "Density must be positive for physics object: " + obj.name};
    }
}

void SceneLoader::validateStaticObject(const StaticObject& obj)
{
    if (!std::filesystem::exists(obj.modelPath)) {
        throw IrrecoverableError{"Model file does not exist: " + obj.modelPath};
    }

    for (float i : obj.scale) {
        if (i <= 0.0F) {
            throw IrrecoverableError{
              "Scale values must be positive for static object: " + obj.name};
        }
    }
}

void SceneLoader::validatePointLight(const PointLightData& light)
{
    // Check that colour values are non-negative
    for (int i = 0; i < 3; ++i) {
        if (light.ambientColour[i] < 0.0F || light.diffuseColour[i] < 0.0F ||
            light.specularColour[i] < 0.0F)
        {
            throw IrrecoverableError{
              "Light colour values must be non-negative"};
        }
    }

    // Check that falloff values are sensible
    if (light.constantFalloff < 0.0F || light.linearFalloff < 0.0F ||
        light.quadraticFalloff < 0.0F)
    {
        throw IrrecoverableError{"Light falloff values must be non-negative"};
    }
}

std::vector<PhysicsEngineReceiverData> SceneLoader::convertToPhysicsObjects(
  const std::vector<PhysicsObject>& physicsObjects)
{
    std::vector<PhysicsEngineReceiverData> physicsEngineObjects;
    physicsEngineObjects.reserve(physicsObjects.size());

    for (const auto& physicsObj : physicsObjects) {
        PhysicsEngineReceiverData physEngineObj{
          .model = std::make_unique<Model>(
            std::filesystem::path{physicsObj.modelPath}
            ),
          .scale =
            linalg::aliases::float3{physicsObj.scale[0], physicsObj.scale[1],
                                  physicsObj.scale[2]},
          .initPos = linalg::aliases::float3{physicsObj.initPos[0],
                                  physicsObj.initPos[1],
                                  physicsObj.initPos[2]},
          .initVel = linalg::aliases::float3{physicsObj.initVel[0],
                                  physicsObj.initVel[1],
                                  physicsObj.initVel[2]},
          .initAngVel = linalg::aliases::float3{physicsObj.initAngVel[0],
                                  physicsObj.initAngVel[1],
                                  physicsObj.initAngVel[2]},
          .density = physicsObj.density
        };

        physicsEngineObjects.push_back(std::move(physEngineObj));
    }

    return physicsEngineObjects;
}

} // namespace Scene
