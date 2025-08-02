#pragma once
#include "linalg.h"

/**
 * @class Quaternion
 * @brief Represents a rotation in 3D space.
 * @ingroup util
 *
 * @details Quaternions are a four-dimensional number system that can be used to
 * represent orientations and rotations of objects in three dimensions. They are
 * composed of a scalar (or real) part \f$w\f$, and a 3D vector (or imaginary)
 * part \f$\mathbf{v} = (x, y, z)\f$. A quaternion \f$q\f$ can be written as
 * \f$q = w + xi + yj + zk\f$.
 *
 * @section Technicality
 * In computer graphics and physics, unit quaternions (where the magnitude is 1)
 * are preferred over Euler angles or rotation matrices for representing
 * rotations. They offer several advantages:
 * - **Avoiding Gimbal Lock:** Unlike Euler angles, quaternions are not
 * susceptible to the loss of a degree of freedom that occurs when axes align.
 * - **Efficient Composition:** Combining two rotations is a single quaternion
 * multiplication, which is computationally cheaper than matrix multiplication.
 * - **Smooth Interpolation:** Spherical Linear Interpolation (SLERP) provides a
 * path of constant angular velocity between two orientations, which is visually
 * smooth and natural.
 *
 * @section Implementation
 * This class serves as a high-level, object-oriented wrapper around the
 * quaternion functionality provided by the `linalg.h` library. It encapsulates
 * the underlying `float4` data structure and exposes a set of methods for
 * creating, manipulating, and applying rotations.
 */
class Quaternion
{
  private:
    /**
     * @brief The underlying data representation: a 4D vector (x, y, z, w).
     */
    linalg::aliases::float4 q;

  public:
    /**
     * @brief Default constructor. Initializes to the identity quaternion
     * (0,0,0,1), representing no rotation.
     */
    Quaternion();

    /**
     * @brief Constructs a quaternion from a `linalg::aliases::float4`.
     * @param quat The source 4D vector.
     */
    explicit Quaternion(const linalg::aliases::float4& quat);

    /**
     * @brief Constructs a quaternion from its four components.
     * @param x The x-component of the vector part.
     * @param y The y-component of the vector part.
     * @param z The z-component of the vector part.
     * @param w The scalar (real) part.
     */
    Quaternion(float x, float y, float z, float w);

    /**
     * @brief Returns the identity quaternion, representing no rotation.
     * @return A quaternion with value (0, 0, 0, 1).
     */
    static Quaternion identity();

    /**
     * @brief Creates a quaternion representing a rotation around a given axis.
     * @param axis The axis of rotation. Must be a unit vector.
     * @param angleRadians The angle of rotation in radians.
     * @return A new quaternion representing the specified rotation.
     */
    static Quaternion fromAxisAngle(const linalg::aliases::float3& axis,
                                    float angleRadians);

    /**
     * @brief Creates a quaternion from a set of Euler angles.
     * @details The rotation is applied in the order: yaw (Z), pitch (Y), roll
     * (X).
     * @param rollRadians Rotation about the X-axis.
     * @param pitchRadians Rotation about the Y-axis.
     * @param yawRadians Rotation about the Z-axis.
     * @return A new quaternion representing the combined rotation.
     */
    static Quaternion fromEulerAngles(float rollRadians, float pitchRadians,
                                      float yawRadians);
    /**
     * @brief Creates a quaternion from a 3x3 rotation matrix.
     * @param rotationMatrix The source rotation matrix.
     * @return A new quaternion equivalent to the matrix's rotation.
     */
    static Quaternion
    fromMatrix(const linalg::aliases::float3x3& rotationMatrix);

    /**
     * @brief Normalizes the quaternion in-place, making its magnitude 1.
     * @details This is essential for ensuring the quaternion represents a pure
     * rotation. It should be called periodically to correct for floating-point
     * drift.
     * @return A reference to this quaternion.
     */
    Quaternion& normalize();

    /**
     * @brief Returns a normalized copy of this quaternion.
     * @return A new quaternion with a magnitude of 1.
     */
    [[nodiscard]] Quaternion normalized() const;

    /**
     * @brief Returns the conjugate of the quaternion.
     * @details The conjugate is found by negating the vector part: \f$q^* = w -
     * xi - yj - zk\f$. For a unit quaternion, the conjugate is equal to its
     * inverse.
     * @return The conjugate quaternion.
     */
    [[nodiscard]] Quaternion conjugate() const;

    /**
     * @brief Returns the inverse of the quaternion.
     * @details For a unit quaternion, this is equivalent to its conjugate. For
     * a non-unit quaternion, the inverse is the conjugate divided by the
     * squared magnitude.
     * @return The inverse quaternion.
     */
    [[nodiscard]] Quaternion inverse() const;

    /**
     * @brief Calculates the magnitude (or length) of the quaternion.
     * @return The magnitude, \f$\sqrt{x^2 + y^2 + z^2 + w^2}\f$.
     */
    [[nodiscard]] float magnitude() const;

    /**
     * @brief Calculates the squared magnitude of the quaternion.
     * @details This is computationally cheaper than `magnitude()` as it avoids
     * a square root. It is often sufficient for comparisons.
     * @return The squared magnitude, \f$x^2 + y^2 + z^2 + w^2\f$.
     */
    [[nodiscard]] float magnitudeSquared() const;

    /**
     * @brief Applies the rotation represented by this quaternion to a 3D
     * vector.
     * @details The rotation is performed using the formula \f$v' = qvq^*\f$,
     * where \f$v\f$ is the vector promoted to a pure quaternion (0, x, y, z).
     * @param vector The vector to be rotated.
     * @return The rotated vector.
     */
    [[nodiscard]] linalg::aliases::float3
    rotate(const linalg::aliases::float3& vector) const;

    /**
     * @brief Converts the quaternion to an equivalent 3x3 rotation matrix.
     * @return A 3x3 rotation matrix.
     */
    [[nodiscard]] linalg::aliases::float3x3 toMatrix3x3() const;

    /**
     * @brief Converts the quaternion to an equivalent 4x4 rotation matrix.
     * @return A 4x4 rotation matrix (with zero translation).
     */
    [[nodiscard]] linalg::aliases::float4x4 toMatrix4x4() const;

    /**
     * @struct RotationMatrices
     * @brief A convenience struct holding both a rotation matrix and its
     * inverse.
     * @ingroup util
     */
    struct RotationMatrices
    {
        linalg::aliases::float4x4 rotation;
        linalg::aliases::float4x4 rotationInverse;
    };

    /**
     * @brief Converts the quaternion to both a 4x4 matrix and its inverse.
     * @details This is more efficient than calculating the matrix and then
     * inverting it separately.
     * @return A `RotationMatrices` struct.
     */
    [[nodiscard]] RotationMatrices toMatrix4x4AndInverse() const;

    /**
     * @brief Converts the quaternion to a set of Euler angles.
     * @details This conversion can be subject to gimbal lock and may not be
     * unique. Its use should be limited, for example, to user interfaces.
     * @return A 3D vector containing (roll, pitch, yaw) in radians.
     */
    [[nodiscard]] linalg::aliases::float3 toEulerAngles() const;

    /**
     * @brief Calculates the 'forward' direction vector (-Z axis) based on the
     * rotation.
     * @return A unit vector representing the local forward direction.
     */
    [[nodiscard]] linalg::aliases::float3 forward() const;

    /**
     * @brief Calculates the 'right' direction vector (+X axis) based on the
     * rotation.
     * @return A unit vector representing the local right direction.
     */
    [[nodiscard]] linalg::aliases::float3 right() const;

    /**
     * @brief Calculates the 'up' direction vector (+Y axis) based on the
     * rotation.
     * @return A unit vector representing the local up direction.
     */
    [[nodiscard]] linalg::aliases::float3 up() const;

    // arithmetic operators
    Quaternion operator*(const Quaternion& other) const;
    Quaternion& operator*=(const Quaternion& other);
    Quaternion operator+(const Quaternion& other) const;
    Quaternion& operator+=(const Quaternion& other);
    Quaternion operator*(float scalar) const;
    Quaternion& operator*=(float scalar);
    Quaternion operator-() const;

    bool operator==(const Quaternion& other) const;
    bool operator!=(const Quaternion& other) const;

    /** @brief Provides const access to the underlying `float4` data. */
    [[nodiscard]] const linalg::aliases::float4& data() const;
    /** @brief Provides mutable access to the underlying `float4` data. */
    [[nodiscard]] linalg::aliases::float4& data();

    /** @name Component Accessors */
    ///@{
    [[nodiscard]] float x() const;
    [[nodiscard]] float y() const;
    [[nodiscard]] float z() const;
    [[nodiscard]] float w() const;

    void setX(float value);
    void setY(float value);
    void setZ(float value);
    void setW(float value);
    ///@}

    /**
     * @brief Performs Spherical Linear Interpolation (SLERP) between two
     * quaternions.
     * @details SLERP provides smooth, constant-speed interpolation along the
     * shortest arc on the 4D unit sphere. It is ideal for high-quality
     * animation but is computationally more expensive than NLERP due to
     * trigonometric functions.
     * @param q1 The starting quaternion.
     * @param q2 The ending quaternion.
     * @param t The interpolation parameter, clamped to the range [0, 1].
     * @return The interpolated quaternion.
     */
    [[nodiscard]] static Quaternion slerp(const Quaternion& q1,
                                          const Quaternion& q2, float t);
    /**
     * @brief Performs Normalized Linear Interpolation (NLERP) between two
     * quaternions.
     * @details NLERP performs a simple linear interpolation of the quaternion
     * components and then re-normalizes the result. It is faster than SLERP but
     * does not guarantee constant angular velocity. It is a very good
     * approximation for small rotational differences.
     * @param q1 The starting quaternion.
     * @param q2 The ending quaternion.
     * @param t The interpolation parameter, clamped to the range [0, 1].
     * @return The interpolated quaternion.
     */
    [[nodiscard]] static Quaternion nlerp(const Quaternion& q1,
                                          const Quaternion& q2, float t);

    /**
     * @brief Calculates the dot product of two quaternions.
     * @param other The other quaternion.
     * @return The dot product.
     */
    [[nodiscard]] float dot(const Quaternion& other) const;

    /**
     * @brief Checks if the quaternion is normalized (has a magnitude of 1).
     * @param tolerance A small tolerance to account for floating-point error.
     * @return `true` if the quaternion is a unit quaternion, `false` otherwise.
     */
    [[nodiscard]] bool isNormalized(float tolerance = 1e-6F) const;

    /**
     * @brief Friend function to allow scalar multiplication from the left.
     */
    friend Quaternion operator*(float scalar, const Quaternion& q);
};
