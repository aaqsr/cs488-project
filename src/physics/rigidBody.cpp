#include "physics/rigidBody.hpp"

#include "frontend/model.hpp"
#include "linalg.h"
#include "physics/constants.hpp"
#include "physics/inertiaTensor.hpp"
#include "util/error.hpp"
#include <algorithm>
#include <memory>
#include <utility>

// RigidBody::RigidBody(
//   std::shared_ptr<const RigidBodyCharacteristics> characteristics,
//   const linalg::aliases::float3& initPos,
//   const linalg::aliases::float3& initVel,
//   const linalg::aliases::float3& initAngVel)
//   : // integrate back in time to discover the initial previous position
//     model{std::move(model)}, scale{scale}, worldPosition{initPos},
//     prevWorldPosition{initPos - initVel * Physics::deltaT +
//                       0.5 * constantAcceleration * Physics::deltaT *
//                         Physics::deltaT},
//     inertiaTensor{this->model.getMeshes(), scale, density},
//     initInvInertia{linalg::inverse(inertiaTensor.initialInertiaMatrix())},
//     angularMomentum{
//       linalg::mul(inertiaTensor.initialInertiaMatrix(), initAngVel)}
// {
//   applyForce()
// }

namespace
{

template <typename T>
std::shared_ptr<T>&& throwIfNullWithMsg(std::shared_ptr<T>&& item,
                                        const std::string& msg)
{
    if (item == nullptr) {
        throw IrrecoverableError{msg};
    }
    return std::move(item);
}

} // namespace

RigidBodyCharacteristics::RigidBodyCharacteristics(
  std::shared_ptr<Model> rigidBodyModel, linalg::aliases::float3 scale,
  float density)
  : model{throwIfNullWithMsg(std::move(rigidBodyModel),
                             "Rigid body's model is nullptr")},
    density{density}, scale{scale},
    inertiaTensor{model->getMeshes(), scale, density},
    initInvInertia{linalg::inverse(inertiaTensor.initialInertiaMatrix())}
{
    // Log model's dimensions
    const std::vector<Mesh>& meshes = model->getMeshes();
    if (meshes.empty()) {
        Logger::GetInstance().log(
          "Warning: Model has no meshes for dimension calculation.");
        return;
    }
    linalg::aliases::float3 firstVertex =
      linalg::cmul(meshes[0].getVertexAtFaceIndex(0).position, scale);
    float minX = firstVertex.x;
    float maxX = firstVertex.x;
    float minY = firstVertex.y;
    float maxY = firstVertex.y;
    float minZ = firstVertex.z;
    float maxZ = firstVertex.z;
    for (const Mesh& mesh : meshes) {
        for (size_t face_idx = 0; face_idx < mesh.getNumFaceIndices();
             ++face_idx)
        {
            linalg::aliases::float3 vertex =
              linalg::cmul(mesh.getVertexAtFaceIndex(face_idx).position, scale);
            minX = std::min(vertex.x, minX);
            maxX = std::max(vertex.x, maxX);
            minY = std::min(vertex.y, minY);
            maxY = std::max(vertex.y, maxY);
            minZ = std::min(vertex.z, minZ);
            maxZ = std::max(vertex.z, maxZ);
        }
    }
    float height = maxY - minY;
    float width = maxX - minX;
    float depth = maxZ - minZ;
    float diameter = (width > depth) ? width : depth;
    Logger::GetInstance().log(
      "Scaled model dimensions - Height: " + std::to_string(height) +
      ", Diameter (approx): " + std::to_string(diameter) + " (Width: " +
      std::to_string(width) + ", Depth: " + std::to_string(depth) + ")");
}

const Inertia& RigidBodyCharacteristics::getInertiaTensor() const
{
    return inertiaTensor;
}

RigidBodyData::RigidBodyData(
  std::shared_ptr<RigidBodyCharacteristics> characteristics,
  const linalg::aliases::float3& initPos,
  const linalg::aliases::float3& initVel,
  const linalg::aliases::float3& initAngVel)
  : characteristics{std::move(characteristics)}, worldPosition{initPos},
    prevWorldPosition{initPos}
{
    if (this->characteristics == nullptr) {
        throw IrrecoverableError{"Rigid Body Characteristics is nullptr"};
    }

    using Physics::RigidBody::deltaT;
    prevWorldPosition = initPos - initVel * deltaT;
    angularMomentum = linalg::mul(
      this->characteristics->getInertiaTensor().initialInertiaMatrix(),
      initAngVel);
}

void RigidBodyData::applyForce(linalg::aliases::float3 force,
                               linalg::aliases::float3 contact)
{
    auto dist = contact - characteristics->getInertiaTensor().getCentreOfMass();
    accumulatedForce += force;
    accumulatedTorque += linalg::cross(dist, force);
}

const linalg::aliases::float3x3&
RigidBodyCharacteristics::getInitInvInertia() const
{
    return initInvInertia;
}

const linalg::aliases::float3& RigidBodyCharacteristics::getScale() const
{
    return scale;
}

const linalg::aliases::float3& RigidBodyData::getWorldPosition() const
{
    return worldPosition;
}

const linalg::aliases::float3& RigidBodyData::getScale() const
{
    return characteristics->getScale();
}

const Quaternion& RigidBodyData::getOrientation() const
{
    return orientation;
}

std::shared_ptr<Model> RigidBodyCharacteristics::getModelPtr() const
{
    return model;
}

std::shared_ptr<Model> RigidBodyData::getModelPtr() const
{
    return characteristics->getModelPtr();
}
