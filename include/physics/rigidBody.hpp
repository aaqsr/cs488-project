#pragma once

#include "physics/AABB.hpp"
#include "physics/inertiaTensor.hpp"
#include "physics/rigidBodyMesh.hpp"
#include "util/quaternion.hpp"

#include <linalg.h>
#include <memory>

// Forward declarations
class Model;
struct Triangle;

/** @defgroup physics Physics Module
 *  Classes related to the physics engine and the simulation of physics.
 */

/**
 * @class RigidBodyCharacteristics
 * @ingroup physics
 *
 * @brief Stores the intrinsic, time-invariant properties of a rigid body.
 *
 * @details This class holds data that does not change during the simulation,
 * such as mass, inertia tensor, and the graphical model. By separating this
 * from the dynamic state, we can use a flyweight pattern where multiple rigid
 * body instances could potentially share the same characteristics, saving
 * memory. This also massively reduces the amount of data that must be actively
 * sent between threads since `RigidBodyData` is synchronised with the render
 * thread every frame.
 */
class RigidBodyCharacteristics
{
    /**
     * @brief A shared pointer to the graphical model used for rendering.
     */
    std::shared_ptr<Model> model;

    /**
     * @brief The uniform density of the body.
     */
    float density = 1.0F;

    /**
     * @brief The initial scale of the model.
     */
    linalg::aliases::float3 scale{1.0F};

    /**
     * @brief The computed inertial properties (mass, COM, inertia tensor) of
     * the body.
     */
    Inertia inertiaTensor;

    /**
     * @brief The pre-computed inverse of the initial inertia tensor in the
     * body's local frame.
     * @details Storing the inverse is a standard performance optimization, as
     * matrix inversion is expensive. In rigid body dynamics, the inverse
     * inertia tensor is used frequently to calculate angular acceleration from
     * torque (\f$\alpha = I^{-1}\tau\f$).
     */
    linalg::aliases::float3x3 initInvInertia{linalg::identity};

    /**
     * @brief The Axis-Aligned Bounding Box that contains the entire rigid body
     * in its local coordinate space.
     *
     * @detail This is an important optimisation as we may then later use this
     * to efficiently compute the world AABB for the body that takes into
     * account its rotation and translation, without having to iterate through
     * its vertices again. (See `AABB::computeMovedAndScaledAABB` for more.)
     */
    AABB localAABB;

    /**
     * @brief Computes the Axis-Aligned Bounding Box that contains the entire
     * rigid body in its local coordinate space.
     *
     * @details This computes the AABB by iterating through all vertices of the
     * rigid body. Whilst an expensive computation, we only pay it once at
     * creation of the body's characteristics.
     */
    void computeLocalAABB();

  public:
    /**
     * @brief Constructs the characteristics by computing the inertia tensor
     * from the model's mesh.
     * @param rigidBodyModel The graphical model whose mesh defines the body's
     * shape.
     * @param scale The scale of the model.
     * @param density The uniform density of the body.
     */
    RigidBodyCharacteristics(std::shared_ptr<Model> rigidBodyModel,
                             linalg::aliases::float3 scale, float density);

    // Deleted copy/move constructors and assignment operators to enforce unique
    // ownership semantics
    RigidBodyCharacteristics(const RigidBodyCharacteristics&) = delete;
    RigidBodyCharacteristics(RigidBodyCharacteristics&&) = default;
    RigidBodyCharacteristics&
    operator=(const RigidBodyCharacteristics&) = delete;
    RigidBodyCharacteristics& operator=(RigidBodyCharacteristics&&) = default;
    ~RigidBodyCharacteristics() = default;

    /** @brief Gets the computed inertia object. */
    [[nodiscard]] const Inertia& getInertiaTensor() const;

    /** @brief Gets the pre-computed inverse inertia tensor in the body's local
     * frame. */
    [[nodiscard]] const linalg::aliases::float3x3& getInitInvInertia() const;

    /** @brief Gets a pointer to the graphical model. */
    [[nodiscard]] std::shared_ptr<Model> getModelPtr() const;

    /** @brief Gets the scale of the body. */
    [[nodiscard]] const linalg::aliases::float3& getScale() const;

    /** @brief Gets the precomputed Axis-Aligned Bounding Box that contains the
     * body in local object coordinates. */
    [[nodiscard]] const AABB& getLocalAABB() const;
};

/**
 * @class RigidBodyData
 * @ingroup physics
 *
 * @brief Represents the dynamic state of a rigid body at a moment in time.
 *
 * @details This class encapsulates the state variables that change
 * frame-to-frame, such as position, orientation, and momentum. The simulation
 * evolves these variables over time.
 *
 * @section Technicality
 * The state is described using linear and angular momentum rather than
 * velocity. This formulation, often used in impulse-based dynamics, simplifies
 * the handling of forces and collisions, as impulses directly modify momentum.
 *
 * We separate the dynamic data of a rigid body from its computation and
 * characteristics, since this data will be often synchronised between threads.
 * Doing this leads to better performance as well as better encapsulation: both
 * the render and physics threads will have access to this class, but both can
 * only perform actions relevant to them.
 *
 * State variables:
 * - Position (`worldPosition`): \f$x(t)\f$
 * - Orientation (`orientation`): \f$q(t)\f$ (a quaternion)
 * - Linear Momentum (`linearMomentum`): \f$P(t) = M v(t)\f$
 * - Angular Momentum (`angularMomentum`): \f$L(t) = I(t) \omega(t)\f$
 *
 * where \f$v\f$ is linear velocity, \f$\omega\f$ is angular velocity, \f$M\f$
 * is mass, and \f$I\f$ is the inertia tensor.
 */
class RigidBodyData
{
    // The main simulation is done in these two classes.
    friend class PhysicsEngine;
    friend class SuperCollider;

    /** @brief Shared pointer to the static characteristics of this body. */
    std::shared_ptr<RigidBodyCharacteristics> characteristics;

    /** @brief The position of the body's origin in world space. */
    linalg::aliases::float3 worldPosition;
    /** @brief The previous position of the body's origin in world space. */
    linalg::aliases::float3 prevWorldPosition;

    /** @brief Accumulated force that is applied and reset at next time-step. */
    linalg::aliases::float3 accumulatedForce = {0.0F, 0.0F, 0.0F};
    /** @brief Accumulated torque that is applied and reset at next time-step.
     */
    linalg::aliases::float3 accumulatedTorque = {0.0F, 0.0F, 0.0F};

    /** @brief The angular momentum of the body in world space.
     * \f$L = I \omega\f$.
     */
    linalg::aliases::float3 angularMomentum = {0.0F, 0.0F, 0.0F};

    /**
     * @brief The orientation of the body in world space, represented by a unit
     * quaternion.
     */
    Quaternion orientation{0.0F, 0.0F, 0.0F, 1.0F};

    /**
     * @brief Cache for the body's triangles transformed into world space.
     * @details This is a performance optimization. Transforming all mesh
     * triangles from body-space to world-space every frame is avoided. The
     * cache is invalidated only when the body's position or orientation
     * changes.
     */
    mutable std::vector<Triangle> cachedTriangles;

    /** @brief Flag indicating if the `cachedTriangles` are up-to-date. */
    mutable bool trianglesCacheValid = false;

  public:
    /** @brief Flag indicating whether the body must be updated by the
     * simulation and drawn by the renderer. */
    bool enabled = true;

    /**
     * @brief Constructs a new rigid body with an initial state.
     * @param characteristics The static properties of the body.
     * @param initPos Initial world position.
     * @param initVel Initial linear velocity.
     * @param initAngVel Initial angular velocity.
     */
    RigidBodyData(std::shared_ptr<RigidBodyCharacteristics> characteristics,
                  const linalg::aliases::float3& initPos = {0.0F, 0.0F, 0.0F},
                  const linalg::aliases::float3& initVel = {0.0F, 0.0F, 0.0F},
                  const linalg::aliases::float3& initAngVel = {0.0F, 0.0F,
                                                               0.0F});

    // Default or deleted special members to manage object lifecycle.
    RigidBodyData(const RigidBodyData&) = default;
    RigidBodyData(RigidBodyData&&) = default;
    RigidBodyData& operator=(const RigidBodyData&) = delete;
    RigidBodyData& operator=(RigidBodyData&&) = delete;
    ~RigidBodyData() = default;

    /**
     * @brief Applies a force at a specific point on the body.
     * @details The force contributes to linear momentum. It also generates a
     * torque
     * \f$r \times F\f$ which contributes to angular momentum, where \f$r\f$ is
     * the vector from the centre of mass to the contact point.
     * @param force The force vector in world space.
     * @param contact The point of application of the force in world space.
     */
    void applyForce(linalg::aliases::float3 force,
                    linalg::aliases::float3 contact);

    /** @brief Gets a pointer to the graphical model. */
    [[nodiscard]] std::shared_ptr<Model> getModelPtr() const;

    /** @brief Gets the world-space position of the body's origin. */
    [[nodiscard]] const linalg::aliases::float3& getWorldPosition() const;
    /** @brief Gets the scale of the body. */
    [[nodiscard]] const linalg::aliases::float3& getScale() const;
    /** @brief Gets the world-space orientation of the body. */
    [[nodiscard]] const Quaternion& getOrientation() const;
    /** @brief Gets the static characteristics of the body. */
    [[nodiscard]] const RigidBodyCharacteristics& getCharacteristics() const;

    /** @brief Calculates and returns the current linear velocity.
     * \f$v = \frac{\text{worldPos} - \text{prevWorldPos}}{\text{mass}}\f$.
     */
    [[nodiscard]] linalg::aliases::float3 getLinearVelocity() const;

    /** @brief Calculates and returns the current angular velocity.
     * \f$\omega = I_{world}^{-1}  L\f$.
     */
    [[nodiscard]] linalg::aliases::float3 getAngularVelocity() const;

    /** @brief Calculates and returns the world-space position of the centre of
     * mass. */
    [[nodiscard]] linalg::aliases::float3 getWorldPosOfCenterOfMass() const;

    /**
     * @brief Calculates the world-space velocity of an arbitrary point on the
     * rigid body.
     *
     * @details The velocity of a point is the sum of the body's linear velocity
     * and the tangential velocity due to its rotation:
     * \f$v_{pt} = v_{com} + (\omega \times r)\f$,
     * where \f$r\f$ is the vector from the centre of mass to the point.
     *
     * @param point The point on the body (in world coordinates) whose velocity
     * is to be found.
     *
     * @return The velocity vector of the point in world space.
     */
    [[nodiscard]] linalg::aliases::float3
    getPointVelocity(const linalg::aliases::float3& point) const;

    /**
     * @brief Retrieves the body's triangles, transformed into world space.
     * @details Uses the internal cache to avoid re-computation if the body has
     * not moved.
     * @return A const reference to the vector of world-space triangles.
     */
    [[nodiscard]] const std::vector<Triangle>& getTriangles() const;

    /**
     * @brief Invalidates the world-space triangle cache.
     * @details This must be called whenever the body's position or orientation
     * is modified.
     */
    void invalidateTriangleCache();

    /**
     * @brief Applies a damping factor to the body's angular momentum.
     * @details This is used to simulate energy loss over time, preventing the
     * simulation from running indefinitely and adding stability.
     */
    void dampenAngularMomentum()
    {
        angularMomentum *= 0.99999F;
        // angularMomentum *= 0.999995F;
    }

    /**
     * @brief Computes the Axis-Aligned Bounding box that would hold the rigid
     * body in world coordinate space.
     * @details This is a wrapper around the `AABB::computeMovedAndScaledAABB()`
     * method on the `AABB` class that allows us to efficiently compute the
     * world space AABB without re-iterating through the body's vertices. See
     * `AABB::computeMovedAndScaledAABB()` for more details.
     */
    AABB computeAABB() const;
};
