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
 * Its main responsibility is to drive the simulation loop each frame.
 *
 * @section Architecture
 * The engine's design incorporates several key patterns:
 * - **Separation of Concerns:** Collision logic is entirely delegated to the
 * static `SuperCollider` class. The engine's role is to manage state and
 * orchestrate the main simulation phases.
 * - **Flyweight Pattern:** It separates intrinsic, static data
 * (`RigidBodyCharacteristics`) from extrinsic, dynamic state (`RigidBodyData`).
 * By storing shared pointers to characteristics, multiple instances of the same
 * object type can share this data, significantly reducing memory usage.
 * - **Asynchronous Creation:** A `Receiver` channel is used to queue requests
 * for new bodies. This decouples the physics thread from other threads,
 * allowing for safe object creation without race conditions.
 *
 * @section Technicality
 * This engine employs a **Position-Based Dynamics (PBD)** approach. Unlike
 * force/momentum-based methods, the state of each body is primarily defined by
 * its current and previous positions and orientations.
 *
 * The core of the simulation is the **Position Verlet integration** scheme
 * implemented in `RigidBodyData::integrate`. Velocity is represented implicitly
 * as the difference between the current and previous positions over the time
 * step. The next position is then extrapolated from these two points, with an
 * added term for acceleration:
 * \f$ x_{n+1} = x_n + (x_n - x_{n-1}) \cdot d_{linear} + \frac{F}{m} \cdot
 * \Delta t^2 \f$ where \f$x_n\f$ is the current position, \f$x_{n-1}\f$ is the
 * previous position, \f$d_{linear}\f$ is a damping factor,
 * \f$F\f$ is the total force, and \f$m\f$ is the mass. A similar Verlet-like
 * scheme is used for rotational motion using quaternions.
 *
 * This method is computationally efficient and is particularly known for its
 * excellent stability, even with large time steps. It is the foundation for
 * modern PBD techniques where constraints are resolved by directly manipulating
 * particle positions.
 *
 * The simulation loop in `updateRigidBodies` proceeds as follows:
 * 1.  **Command Processing:** Create new bodies from the channel.
 * 2.  **State Integration:** Call `integrate` on each body to advance its
 * position based on its history and applied forces.
 * 3.  **Collision Handling:** Invoke `SuperCollider` to detect and resolve
 * penetrations. In a PBD context, this resolution often involves directly
 * correcting the positions of colliding bodies.
 *
 * @section Data and Code Sources
 * Position-Based Dynamics and Verlet integration are foundational topics in
 * physical simulation. The techniques are detailed in numerous sources, with
 * seminal work including the paper "Position Based Dynamics" by Matthias Müller
 * et al. (2007) (who also was a major contributor to the water simulation
 * paper we used. Thanks Matthias!)
 *
 * The quaternion math used to compute rotational dynamics was inspired by
 * "Robust rotational-velocity-Verlet integration methods" by Dmitri Rozmanov et
 * al. (2010), as well as the "Physics - Kinematics - Angular Velocity" article
 * by Euclidean Space,
 * (https://www.euclideanspace.com/physics/kinematics/angularvelocity/)
 */
class PhysicsEngine
{
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
     * @details This method orchestrates the application of forces and the
     * integration of the body's state using a position-based (Verlet) scheme.
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
     * It orchestrates the creation of new bodies, state integration, and
     * collision handling.
     * @param rigidBodies A vector of all dynamic rigid body data to be updated.
     * @param prevRigidBodies A const reference to the state of the bodies from
     * the previous frame.
     * @param keepBodiesWithinWaterPool Should bodies be kept in the water pool
     */
    void updateRigidBodies(std::vector<RigidBodyData>& rigidBodies,
                           const std::vector<RigidBodyData>& prevRigidBodies,
                           bool keepBodiesWithinWaterPool);
};
