#pragma once

#include "linalg.h"
#include "shader.hpp"
#include "window.hpp"

class Camera
{
    float fov = 45.0F;
    float aspectRatio = (float)Window::width / (float)Window::height;

    // TODO: Should near plane be this close? Or is this far enough? Value
    // stolen from assignment code.
    float nearPlane = 5e-5F;
    float farPlane = 100.0F;

    linalg::aliases::float3 up{0.0F, 1.0F, 0.0F};

    linalg::aliases::float3 pos{0.0F, 0.0F, 1.5F};
    linalg::aliases::float3 lookAt{0.0F, 0.0F, 0.0F};

    linalg::aliases::float4x4 perspectiveMatrix;
    linalg::aliases::float4x4 lookAtMatrix;

    void updateLookAt();

  public:
    Camera();

    // void move(linalg::aliases::float3 displacement);

    [[nodiscard]] const linalg::aliases::float4x4& getPerspectiveMatrix() const
    {
        return perspectiveMatrix;
    }
    [[nodiscard]] const linalg::aliases::float4x4& getLookAtMatrix() const
    {
        return lookAtMatrix;
    }

    void setUniforms(Shader& shader) const;
};
