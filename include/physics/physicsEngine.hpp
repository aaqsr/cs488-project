#pragma once

#include <linalg.h>

#include <memory>
#include <vector>

// Forward declarations to reduce header dependencies
class Model;
class RigidBodyCharacteristics;
class RigidBodyData;
template <typename T>
class Receiver;

/**
 * @struct PhysicsEngineReceiverData
 * @brief A data structure for passing information to create a new rigid body.
 * @ingroup physics
 *
 * @details This acts as a command object, decoupling the client code that
 * spawns objects from the physics engine itself. It bundles all necessary
 * initial parameters. This architecture allows for object creation requests to
 * be queued and processed safely by the physics thread at the start of a
 * simulation step.
 */
struct PhysicsEngineReceiverData
{
    std::shared_ptr<Model> model;
    linalg::aliases::float3 scale{1.0F};
    linalg::aliases::float3 initPos{0.0F};
    linalg::aliases::float3 initVel{0.0F};
    linalg::aliases::float3 initAngVel{0.0F};
    linalg::aliases::float3 initForce{0.0F};
    linalg::aliases::float3 initForceContact{0.0F};
    float density = 1.0F;
};

/**
 * @class PhysicsEngine
 * @brief Manages and simulates a collection of rigid bodies.
 * @ingroup physics
 *
 * @details The engine is the top-level orchestrator for the physics simulation.
 * Its main responsibility is to drive the simulation loop each frame, which
 * involves creating new bodies, applying forces, integrating motion, and
 * invoking the collision system.
 *
 * The engine's responsibilities have been refined. It no longer contains
 * collision logic directly. Instead, it delegates all collision detection and
 * resolution tasks to the static `SuperCollider` class. This separation of
 * concerns improves modularity and clarity.
 */
class PhysicsEngine
{
    /**
     * @brief Damping factor to gradually reduce linear velocity over time,
     * simulating friction/drag.
     */
    constexpr static float linearDamping = 1.0F;

    /**
     * @brief Damping factor to gradually reduce angular velocity over time.
     */
    constexpr static float angularDamping = 1.0F;

    /**
     * @brief A communication channel to receive data for creating new rigid
     * bodies asynchronously.
     */
    Receiver<std::vector<PhysicsEngineReceiverData>>& channel;

    /**
     * @brief A collection of the static, intrinsic properties for all unique
     * body types. This allows for memory sharing via a flyweight pattern.
     */
    std::vector<std::shared_ptr<RigidBodyCharacteristics>>
      rigidBodyCharacteristics;

    /**
     * @brief Simulates a single rigid body's motion for one time step.
     * @param out The output state of the rigid body after the simulation step.
     * @param prev The state of the rigid body from the previous time step.
     */
    void simulateRigidBody(RigidBodyData& out, const RigidBodyData& prev);

  public:
    /**
     * @brief Constructs the PhysicsEngine.
     * @param recv A reference to the receiver end of the communication channel
     * for new objects.
     */
    PhysicsEngine(Receiver<std::vector<PhysicsEngineReceiverData>>& recv);

    /**
     * @brief The main update function for the physics simulation.
     * @details This method advances the entire physics world by one time step.
     * It performs the following steps:
     * 1. Receives and creates any new rigid bodies requested via the channel.
     * 2. Iterates through all bodies, calling `simulateRigidBody` to apply
     * forces and integrate motion.
     * 3. Invokes the `SuperCollider` to handle boundary constraints and
     * inter-object collisions.
     * @param rigidBodies A vector of all dynamic rigid body data to be updated.
     * @param prevRigidBodies A const reference to the state of the bodies from
     * the previous frame.
     */
    void updateRigidBodies(std::vector<RigidBodyData>& rigidBodies,
                           const std::vector<RigidBodyData>& prevRigidBodies);
};
