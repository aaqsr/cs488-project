#include "physics/rigidBody.hpp"

#include "frontend/model.hpp"
#include "physics/constants.hpp"
#include "physics/inertiaTensor.hpp"
#include "physics/rigidBodyMesh.hpp"
#include "util/error.hpp"
#include "util/logger.hpp"

#include <linalg.h>

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
    computeLocalAABB();

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
    auto dist = contact - getWorldPosOfCenterOfMass();
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

const RigidBodyCharacteristics& RigidBodyData::getCharacteristics() const
{
    return *characteristics;
}

std::shared_ptr<Model> RigidBodyCharacteristics::getModelPtr() const
{
    return model;
}

std::shared_ptr<Model> RigidBodyData::getModelPtr() const
{
    return characteristics->getModelPtr();
}
linalg::aliases::float3 RigidBodyData::getLinearVelocity() const
{
    return (worldPosition - prevWorldPosition) / Physics::RigidBody::deltaT;
}

linalg::aliases::float3 RigidBodyData::getAngularVelocity() const
{
    // ω = I^(-1) * L
    auto R = orientation.toMatrix3x3();
    linalg::aliases::float3x3 worldInvInertia =
      linalg::mul(linalg::mul(R, characteristics->getInitInvInertia()),
                  linalg::transpose(R));
    return linalg::mul(worldInvInertia, angularMomentum);
}

linalg::aliases::float3 RigidBodyData::getWorldPosOfCenterOfMass() const
{
    linalg::aliases::float4x4 transformMatrix =
      linalg::mul(linalg::mul(linalg::translation_matrix(worldPosition),
                              orientation.toMatrix4x4()),
                  linalg::scaling_matrix(characteristics->getScale()));
    linalg::aliases::float4 worldCenterOfMass = linalg::mul(
      transformMatrix,
      linalg::aliases::float4{
        characteristics->getInertiaTensor().getCentreOfMass(), 1.0F});
    return worldCenterOfMass.xyz();
}

linalg::aliases::float3
RigidBodyData::getPointVelocity(const linalg::aliases::float3& point) const
{
    linalg::aliases::float3 linearVel = getLinearVelocity();
    linalg::aliases::float3 angularVel = getAngularVelocity();
    linalg::aliases::float3 r = point - getWorldPosOfCenterOfMass();
    return linearVel + linalg::cross(angularVel, r);
}

void RigidBodyData::invalidateTriangleCache()
{
    trianglesCacheValid = false;
}

const std::vector<Triangle>& RigidBodyData::getTriangles() const
{
    if (trianglesCacheValid) {
        return cachedTriangles;
    }

    cachedTriangles.clear();

    std::shared_ptr<Model> model = characteristics->getModelPtr();
    if (!model) {
        throw IrrecoverableError{
          "[RigidBodyData::getTriangles()] WHERE DID THE MODEL GO"};
    }

    linalg::aliases::float4x4 transformMatrix =
      linalg::mul(linalg::mul(linalg::translation_matrix(worldPosition),
                              orientation.toMatrix4x4()),
                  linalg::scaling_matrix(characteristics->getScale()));

    // TODO: Surely instead of reading them from the mesh every single time I
    // can just read once and transform each time right? surely right? or
    // actually i guess this is pretty much just doing that anyways...
    for (const Mesh& mesh : model->getMeshes()) {
        uint32_t numIndices = mesh.getNumFaceIndices();

        for (uint32_t i = 0; i < numIndices; i += 3) {
            if (i + 2 >= numIndices) {
                break;
            }

            const Vertex& v0 = mesh.getVertexAtFaceIndex(i);
            const Vertex& v1 = mesh.getVertexAtFaceIndex(i + 1);
            const Vertex& v2 = mesh.getVertexAtFaceIndex(i + 2);

            // to world space!
            linalg::aliases::float4 worldV0 =
              linalg::mul(transformMatrix,
                          linalg::aliases::float4{v0.position.x, v0.position.y,
                                                  v0.position.z, 1.0F});
            linalg::aliases::float4 worldV1 =
              linalg::mul(transformMatrix,
                          linalg::aliases::float4{v1.position.x, v1.position.y,
                                                  v1.position.z, 1.0F});
            linalg::aliases::float4 worldV2 =
              linalg::mul(transformMatrix,
                          linalg::aliases::float4{v2.position.x, v2.position.y,
                                                  v2.position.z, 1.0F});

            linalg::aliases::float3 worldPos0{worldV0.x, worldV0.y, worldV0.z};
            linalg::aliases::float3 worldPos1{worldV1.x, worldV1.y, worldV1.z};
            linalg::aliases::float3 worldPos2{worldV2.x, worldV2.y, worldV2.z};

            cachedTriangles.emplace_back(worldPos0, worldPos1, worldPos2);
        }
    }

    trianglesCacheValid = true;
    return cachedTriangles;
}

void RigidBodyCharacteristics::computeLocalAABB()
{
    const std::vector<Mesh>& meshes = model->getMeshes();
    if (meshes.empty()) {
        localAABB =
          AABB{linalg::aliases::float3{0.0F}, linalg::aliases::float3{0.0F}};
        return;
    }

    linalg::aliases::float3 firstVertex =
      linalg::cmul(meshes[0].getVertexAtFaceIndex(0).position, scale);
    localAABB = AABB{firstVertex, firstVertex};

    for (const Mesh& mesh : meshes) {
        for (size_t face_idx = 0; face_idx < mesh.getNumFaceIndices();
             ++face_idx)
        {
            linalg::aliases::float3 scaledVertex =
              linalg::cmul(mesh.getVertexAtFaceIndex(face_idx).position, scale);
            localAABB.expand(scaledVertex);
        }
    }
}
const AABB& RigidBodyCharacteristics::getLocalAABB() const
{
    return localAABB;
}
AABB RigidBodyData::computeAABB() const
{
    const AABB& localAABB = characteristics->getLocalAABB();
    linalg::aliases::float3x3 rotationMatrix = orientation.toMatrix3x3();

    return localAABB.computeMovedAndScaledAABB(worldPosition, rotationMatrix);
}
