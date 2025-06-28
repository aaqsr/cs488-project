#pragma once
#include <linalg.h>

class Quaternion
{
  private:
    linalg::aliases::float4 q; // (x, y, z, w) representation

  public:
    Quaternion();
    explicit Quaternion(const linalg::aliases::float4& quat);
    Quaternion(float x, float y, float z, float w);

    static Quaternion identity();
    static Quaternion fromAxisAngle(const linalg::aliases::float3& axis,
                                    float angleRadians);
    static Quaternion fromEulerAngles(float yawRadians, float pitchRadians,
                                      float rollRadians = 0.0F);
    static Quaternion
    fromMatrix(const linalg::aliases::float3x3& rotationMatrix);

    // basic
    Quaternion& normalize();
    [[nodiscard]] Quaternion normalized() const;
    [[nodiscard]] Quaternion conjugate() const;
    [[nodiscard]] Quaternion inverse() const;
    [[nodiscard]] float magnitude() const;
    [[nodiscard]] float magnitudeSquared() const;

    // rotation
    [[nodiscard]] linalg::aliases::float3
    rotate(const linalg::aliases::float3& vector) const;
    [[nodiscard]] linalg::aliases::float3x3 toMatrix3x3() const;
    [[nodiscard]] linalg::aliases::float4x4 toMatrix4x4() const;

    // axis and angle
    [[nodiscard]] linalg::aliases::float3 axis() const;
    [[nodiscard]] float angle() const;

    // direction vectors
    [[nodiscard]] linalg::aliases::float3 forward() const;
    [[nodiscard]] linalg::aliases::float3 right() const;
    [[nodiscard]] linalg::aliases::float3 up() const;

    // arithmetic
    Quaternion operator*(const Quaternion& other) const;
    Quaternion& operator*=(const Quaternion& other);
    Quaternion operator+(const Quaternion& other) const;
    Quaternion& operator+=(const Quaternion& other);
    Quaternion operator*(float scalar) const;
    Quaternion& operator*=(float scalar);
    Quaternion operator-() const;

    bool operator==(const Quaternion& other) const;
    bool operator!=(const Quaternion& other) const;

    // underlying data
    [[nodiscard]] const linalg::aliases::float4& data() const;
    [[nodiscard]] linalg::aliases::float4& data();

    // components
    [[nodiscard]] float x() const;
    [[nodiscard]] float y() const;
    [[nodiscard]] float z() const;
    [[nodiscard]] float w() const;

    void setX(float value);
    void setY(float value);
    void setZ(float value);
    void setW(float value);

    // interpolation
    [[nodiscard]] static Quaternion slerp(const Quaternion& q1,
                                          const Quaternion& q2, float t);
    [[nodiscard]] static Quaternion nlerp(const Quaternion& q1,
                                          const Quaternion& q2, float t);

    // utility fns
    [[nodiscard]] float dot(const Quaternion& other) const;
    [[nodiscard]] bool isNormalized(float tolerance = 1e-6F) const;

    // friend fn for scalar multiplication from left
    friend Quaternion operator*(float scalar, const Quaternion& q);
};
