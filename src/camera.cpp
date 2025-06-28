#include "camera.hpp"

#include <linalg.h>

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

float4x4 computeLookAtMatrix(const float3& _eye, const float3& _center,
                             const float3& _up)
{
    // transformation to the camera coordinate
    float4x4 m;
    const float3 f = normalize(_center - _eye);
    const float3 upp = normalize(_up);
    const float3 s = normalize(cross(f, upp));
    const float3 u = cross(s, f);

    m[0] = {s.x, s.y, s.z, 0.0F};
    m[1] = {u.x, u.y, u.z, 0.0F};
    m[2] = {-f.x, -f.y, -f.z, 0.0F};
    m[3] = {0.0F, 0.0F, 0.0F, 1.0F};
    m = transpose(m);

    // translation according to the camera location
    const float4x4 t = float4x4{
      {   1.0F,    0.0F,    0.0F, 0.0F},
      {   0.0F,    1.0F,    0.0F, 0.0F},
      {   0.0F,    0.0F,    1.0F, 0.0F},
      {-_eye.x, -_eye.y, -_eye.z, 1.0F}
    };

    m = mul(m, t);
    return m;
}
} // namespace

void Camera::updateLookAt()
{
    lookAtMatrix = computeLookAtMatrix(pos, lookAt, up);
}

Camera::Camera()
  : perspectiveMatrix{computePerspectiveMatrix(fov, aspectRatio, nearPlane,
                                               farPlane)},
    lookAtMatrix{computeLookAtMatrix(pos, lookAt, up)}
{
}

void Camera::setUniforms(Shader& shader) const
{
    // TODO: projection doesn't change?? Why send each time? Can we not?
    shader.setUniform("projection", perspectiveMatrix);
    shader.setUniform("view", lookAtMatrix);
}

// void Camera::moveY(float dist)
// {
//     pos += dist;
//     lookAt += dist;
//     updateLookAt();
// }
