#include "quaternion.hpp"
#include <cmath>

using namespace linalg::aliases;

Quaternion::Quaternion() : q{0.0F, 0.0F, 0.0F, 1.0F}
{
} // Identity quaternion

Quaternion::Quaternion(const float4& quat) : q(quat)
{
}

Quaternion::Quaternion(float x, float y, float z, float w) : q{x, y, z, w}
{
}

Quaternion Quaternion::identity()
{
    return {};
}

Quaternion Quaternion::fromAxisAngle(const float3& axis, float angleRadians)
{
    return Quaternion(linalg::rotation_quat(axis, angleRadians));
}

Quaternion Quaternion::fromEulerAngles(float yawRadians, float pitchRadians,
                                       float rollRadians)
{
    // quaternions for each axis using linalg::rotation_quat
    const float3 xAxis{1.0F, 0.0F, 0.0F};
    const float3 yAxis{0.0F, 1.0F, 0.0F};
    const float3 zAxis{0.0F, 0.0F, 1.0F};

    const float4 rollQuat = linalg::rotation_quat(xAxis, rollRadians);
    const float4 pitchQuat = linalg::rotation_quat(yAxis, pitchRadians);
    const float4 yawQuat = linalg::rotation_quat(zAxis, yawRadians);

    // Combine rotations: Yaw * Pitch * Roll
    const float4 combined =
      linalg::qmul(linalg::qmul(yawQuat, pitchQuat), rollQuat);

    return Quaternion(combined);
}

Quaternion Quaternion::fromMatrix(const float3x3& rotationMatrix)
{
    return Quaternion(linalg::rotation_quat(rotationMatrix));
}

Quaternion& Quaternion::normalize()
{
    q = linalg::normalize(q);
    return *this;
}

Quaternion Quaternion::normalized() const
{
    return Quaternion(linalg::normalize(q));
}

Quaternion Quaternion::conjugate() const
{
    return Quaternion(linalg::qconj(q));
}

Quaternion Quaternion::inverse() const
{
    return Quaternion(linalg::qinv(q));
}

float Quaternion::magnitude() const
{
    return linalg::length(q);
}

float Quaternion::magnitudeSquared() const
{
    return linalg::length2(q);
}

float3 Quaternion::rotate(const float3& vector) const
{
    return linalg::qrot(q, vector);
}

float3x3 Quaternion::toMatrix3x3() const
{
    return linalg::qmat(q);
}

float4x4 Quaternion::toMatrix4x4() const
{
    return linalg::rotation_matrix(q);
}

float3 Quaternion::axis() const
{
    return linalg::qaxis(q);
}

float Quaternion::angle() const
{
    return linalg::qangle(q);
}

float3 Quaternion::forward() const
{
    // forward is negative Z direction
    return -linalg::qzdir(q);
}

float3 Quaternion::right() const
{
    // right is positive X direction
    return linalg::qxdir(q);
}

float3 Quaternion::up() const
{
    // up is positive Y direction
    return linalg::qydir(q);
}

Quaternion Quaternion::operator*(const Quaternion& other) const
{
    return Quaternion(linalg::qmul(q, other.q));
}

Quaternion& Quaternion::operator*=(const Quaternion& other)
{
    q = linalg::qmul(q, other.q);
    return *this;
}

Quaternion Quaternion::operator+(const Quaternion& other) const
{
    return Quaternion(q + other.q);
}

Quaternion& Quaternion::operator+=(const Quaternion& other)
{
    q = q + other.q;
    return *this;
}

Quaternion Quaternion::operator*(float scalar) const
{
    return Quaternion(q * scalar);
}

Quaternion& Quaternion::operator*=(float scalar)
{
    q = q * scalar;
    return *this;
}

Quaternion Quaternion::operator-() const
{
    return Quaternion(-q);
}

bool Quaternion::operator==(const Quaternion& other) const
{
    return q == other.q;
}

bool Quaternion::operator!=(const Quaternion& other) const
{
    return q != other.q;
}

const float4& Quaternion::data() const
{
    return q;
}

float4& Quaternion::data()
{
    return q;
}

float Quaternion::x() const
{
    return q.x;
}
float Quaternion::y() const
{
    return q.y;
}
float Quaternion::z() const
{
    return q.z;
}
float Quaternion::w() const
{
    return q.w;
}

void Quaternion::setX(float value)
{
    q.x = value;
}
void Quaternion::setY(float value)
{
    q.y = value;
}
void Quaternion::setZ(float value)
{
    q.z = value;
}
void Quaternion::setW(float value)
{
    q.w = value;
}

Quaternion Quaternion::slerp(const Quaternion& q1, const Quaternion& q2,
                             float t)
{
    return Quaternion(linalg::qslerp(q1.q, q2.q, t));
}

Quaternion Quaternion::nlerp(const Quaternion& q1, const Quaternion& q2,
                             float t)
{
    return Quaternion(linalg::qnlerp(q1.q, q2.q, t));
}

float Quaternion::dot(const Quaternion& other) const
{
    return linalg::dot(q, other.q);
}

bool Quaternion::isNormalized(float tolerance) const
{
    const float magSq = magnitudeSquared();
    return std::abs(magSq - 1.0F) < tolerance;
}

Quaternion operator*(float scalar, const Quaternion& q)
{
    return q * scalar;
}
