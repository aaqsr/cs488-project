#pragma once

#include "quaternion.hpp"
#include "shader.hpp"
#include "window.hpp"

#include <linalg.h>

class Camera
{
    float fov = 45.0F;
    float aspectRatio = (float)Window::width / (float)Window::height;

    // TODO: Should near plane be this close? Or is this far enough? Value
    // stolen from assignment code.
    float nearPlane = 5e-5F;
    // float nearPlane = 0.1F;
    float farPlane = 100.0F;

    linalg::aliases::float3 position{0.0F, 0.0F, 2.0F};
    Quaternion orientation{0.0F, 0.0F, 0.0F, 1.0F};

    linalg::aliases::float4x4 perspectiveMatrix;
    linalg::aliases::float4x4 viewMatrix;

    // cached for performance for things that need it, hence mutable
    // (TODO: does this make a difference?)
    mutable bool vectorsNeedUpdate = true;
    mutable linalg::aliases::float3 front{0.0F, 0.0F, -1.0F};
    mutable linalg::aliases::float3 up{0.0F, 1.0F, 0.0F};
    mutable linalg::aliases::float3 right{1.0F, 0.0F, 0.0F};

    void updateViewMatrix();
    void updateDirectionVectors() const;

  public:
    Camera();

    void setPosition(const linalg::aliases::float3& pos);
    void move(const linalg::aliases::float3& displacement);
    void setOrientation(const Quaternion& quat);
    void rotate(const Quaternion& deltaRotation);
    void rotateAroundAxis(const linalg::aliases::float3& axis,
                          float angleRadians);

    [[nodiscard]] const linalg::aliases::float3& getPosition() const
    {
        return position;
    }
    [[nodiscard]] const Quaternion& getOrientation() const
    {
        return orientation;
    }
    [[nodiscard]] const linalg::aliases::float3& getFront() const
    {
        updateDirectionVectors();
        return front;
    }
    [[nodiscard]] const linalg::aliases::float3& getUp() const
    {
        updateDirectionVectors();
        return up;
    }
    [[nodiscard]] const linalg::aliases::float3& getRight() const
    {
        updateDirectionVectors();
        return right;
    }
    [[nodiscard]] const linalg::aliases::float4x4& getPerspectiveMatrix() const
    {
        return perspectiveMatrix;
    }
    [[nodiscard]] const linalg::aliases::float4x4& getViewMatrix() const
    {
        return viewMatrix;
    }

    void setUniforms(Shader::BindObject& shader) const;
};
