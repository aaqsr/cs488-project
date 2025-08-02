#pragma once

#include <linalg.h>

/**
 * @namespace Physics
 * @brief Defines global constants and configurable parameters for the physics
 * simulation.
 * @ingroup physics
 *
 * @details This namespace centralises fundamental values used across the
 * physics engine, including gravitational acceleration and simulation time
 * steps. Grouping these constants ensures consistency and simplifies tuning and
 * maintenance.
 */
namespace Physics
{

/**
 * @brief The standard acceleration due to gravity, \f$ g \f$.
 */
constexpr float gravitationalAccelerationMagnitude = 9.80665F;

/**
 * @brief The gravitational acceleration vector.
 * @details A constant vector representing gravity acting downwards along the
 * negative y-axis.
 */
const linalg::aliases::float3 gravitationalAcceleration{
  0.0F, -gravitationalAccelerationMagnitude, 0.0F};

/**
 * @namespace WaterSim
 * @brief Defines constants specific to the water simulation component.
 */
namespace WaterSim
{

/**
 * @brief The size of a single cell in the water simulation grid.
 */
constexpr static float cellSize = 0.05F;

/**
 * @brief The maximum depth of the water in the simulation.
 */
constexpr static float maxDepth = 1.5F;

/**
 * @brief The time step (\f$ \Delta t \f$) for the water simulation.
 * @details A very small time step is chosen to satisfy the
 * Courant–Friedrichs–Lewy (CFL) condition for the fluid simulation, ensuring
 * numerical stability. **NOTE:** the simulation may not actually run this time
 * step, as it internally employs adaptive time stepping to split the time step
 * up. The simulation does guarantee that the total time stepped through at the
 * end of update will equal this value however. See `WaterSimulation`'s method
 * `WaterSimulation::calculateOptimalSubSteps` for more.
 */
constexpr static float deltaT = 0.0002F;

} // namespace WaterSim

/**
 * @namespace RigidBody
 * @brief Defines constants specific to the rigid body simulation component.
 */
namespace RigidBody
{

/**
 * @brief The time step (\f$ \Delta t \f$) for the rigid body simulation.
 * @details This is explicitly set to be the same as the water simulation's time
 * step. This is due to our **tightly coupled** simulation approach, where both
 * the fluid and rigid body systems are advanced by the same small increment.
 * This is necessary to ensure stable and accurate two-way interactions between
 * the bodies and the fluid.
 */
constexpr static float deltaT = WaterSim::deltaT;

} // namespace RigidBody

} // namespace Physics
