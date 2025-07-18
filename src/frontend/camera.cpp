#include "frontend/camera.hpp"
#include "linalg.h"
#include <numbers>

namespace
{
using namespace linalg::aliases;

constexpr float PI = std::numbers::pi_v<float>;
constexpr float DegToRad = PI / 180.0F;

float4x4 computePerspectiveMatrix(float fovy, float aspect, float zNear,
                                  float zFar)
{
    float4x4 m;
    const float f = 1.0F / (tan(fovy * DegToRad / 2.0F));
    m[0] = {f / aspect, 0.0F, 0.0F, 0.0F};
    m[1] = {0.0F, f, 0.0F, 0.0F};
    m[2] = {0.0F, 0.0F, (zFar + zNear) / (zNear - zFar), -1.0F};
    m[3] = {0.0F, 0.0F, (2.0F * zFar * zNear) / (zNear - zFar), 0.0F};
    return m;
}

float4x4 computeViewMatrix(const float3& position,
                           const Quaternion& orientation)
{
    const float4x4 rotationMatrix = orientation.toMatrix4x4();

    // translation according to the camera location
    const float4x4 translationMatrix = float4x4{
      {       1.0F,        0.0F,        0.0F, 0.0F},
      {       0.0F,        1.0F,        0.0F, 0.0F},
      {       0.0F,        0.0F,        1.0F, 0.0F},
      {-position.x, -position.y, -position.z, 1.0F}
    };

    // view matrix is the inverse of the camera transform
    // rotation matrix is orthogonal, so its inverse is just its transpose
    const float4x4 rotationTranspose = transpose(rotationMatrix);

    return mul(rotationTranspose, translationMatrix);
}

} // namespace

Camera::Camera()
  : perspectiveMatrix{
      computePerspectiveMatrix(fov, aspectRatio, nearPlane, farPlane)}
// init with identity quaternion
{
    updateViewMatrix();
}

void Camera::updateViewMatrix()
{
    viewMatrix = computeViewMatrix(position, orientation);
}

void Camera::updateDirectionVectors() const
{
    if (!vectorsNeedUpdate) {
        return;
    }

    front = orientation.forward();
    right = orientation.right();
    up = orientation.up();

    vectorsNeedUpdate = false;
}

void Camera::setPosition(const linalg::aliases::float3& pos)
{
    position = pos;
    updateViewMatrix();
}

void Camera::move(const linalg::aliases::float3& displacement)
{
    position = position + displacement;
    updateViewMatrix();
}

void Camera::setOrientation(const Quaternion& quat)
{
    orientation = quat.normalized();
    vectorsNeedUpdate = true;
    updateViewMatrix();
}

void Camera::rotate(const Quaternion& deltaRotation)
{
    orientation = (deltaRotation * orientation).normalized();
    vectorsNeedUpdate = true;
    updateViewMatrix();
}

void Camera::rotateAroundAxis(const linalg::aliases::float3& axis,
                              float angleRadians)
{
    const Quaternion axisRotation =
      Quaternion::fromAxisAngle(axis, angleRadians);
    rotate(axisRotation);
}

void Camera::setUniforms(Shader::BindObject& shader) const
{
    // TODO: probably doesn't need to be done each frame if values don't change?
    shader.setUniform("projection", perspectiveMatrix);
    shader.setUniform("view", viewMatrix);
    shader.setUniform("viewPos", position);
}
void Camera::updatePerspectiveMatrix()
{
    perspectiveMatrix =
      computePerspectiveMatrix(fov, aspectRatio, nearPlane, farPlane);
}

void Camera::updateAspectRatio(int width, int height)
{
    aspectRatio = static_cast<float>(width) / static_cast<float>(height);
    updatePerspectiveMatrix();
}
