#include "util/quaternion.hpp"
#include "linalg.h"
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

Quaternion Quaternion::fromEulerAngles(float rollRadians, float pitchRadians,
                                       float yawRadians)
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

namespace
{
linalg::aliases::float4x4 toHomogenousMatrix(const linalg::aliases::float3x3& m)
{
    return linalg::aliases::float4x4{
      {m[0], 0.0F},
      {m[1], 0.0F},
      {m[2], 0.0F},
      {0.0F, 0.0F, 0.0F, 1.0F}
    };
}
} // namespace

Quaternion::RotationMaterices Quaternion::toMatrix4x4AndInverse() const
{
    const linalg::aliases::float3x3 inhomogenousRotation = linalg::qmat(q);
    // orthogonal matrix
    const linalg::aliases::float3x3 inhomogenousRotationInverse =
      linalg::transpose(inhomogenousRotation);
    return {toHomogenousMatrix(inhomogenousRotation),
            toHomogenousMatrix(inhomogenousRotationInverse)};
}

linalg::aliases::float3 Quaternion::toEulerAngles() const
{
    float q2sqr = q.z * q.z;
    const float t0 = (-2.0F * (q2sqr + q.w * q.w)) + 1.0F;
    const float t1 = +2.0F * (q.y * q.z + q.x * q.w);
    const float t3 = +2.0F * (q.z * q.w + q.x * q.y);
    const float t4 = (-2.0F * (q.y * q.y + q2sqr)) + 1.0F;

    float t2 = -2.0F * (q.y * q.w - q.x * q.z);

    t2 = t2 > 1.0F ? 1.0F : t2;
    t2 = t2 < -1.0F ? -1.0F : t2;

    const float roll = atan2(t3, t4);
    const float pitch = asin(t2);
    const float yaw = atan2(t1, t0);

    return {roll, pitch, yaw};
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
