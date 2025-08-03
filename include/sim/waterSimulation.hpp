#pragma once

#include "physics/constants.hpp"
#include "sim/staggeredGrid.hpp"
#include "sim/waterHeightGrid.hpp"

#include <linalg.h>

struct SubTriangle;
class RigidBodyData;

/** @defgroup sim Water Simulation
 *  Classes that are related to the main height-field water simulation.
 */

/**
 * @class WaterSimulation
 * @brief Manages a 2D heightfield fluid simulation with two-way rigid body
 * coupling and adaptive sub-stepping.
 * @ingroup sim
 *
 * @details This class implements a fluid simulation based on a 2D heightfield
 * grid. This is an efficient technique for simulating large bodies of water,
 * such as pools or oceans, where the fluid depth is much smaller than its
 * horizontal extent. The simulation solves a simplified version of the
 * Navier-Stokes equations, specifically the shallow water equations, which
 * assume that vertical fluid motion is negligible compared to horizontal
 * motion.
 *
 * @section Technicality
 * The core of the simulation is a **staggered grid** (specifically, an Arakawa
 * C-grid) for storing fluid quantities. This is a standard numerical method
 * that provides superior stability and accuracy for gradient and divergence
 * calculations compared to a simple collocated grid.
 * - **Height and Pressure:** The water height (`h`) is stored at the centre of
 * each grid cell. In the shallow water model, the pressure at the bottom of a
 * fluid column is hydrostatic and directly proportional to this height.
 * - **Velocities:** The horizontal velocity components (`u` for the
 * x-direction, `v` for the z-direction) are "staggered", meaning they are
 * stored at the faces of the grid cells. The `u` velocity is stored on the
 * vertical faces (left/right), and `v` is stored on the horizontal faces
 * (top/bottom).
 *
 * @subsection Adaptive Sub-stepping and CFL Condition
 * To ensure numerical stability, this simulation employs an **adaptive
 * sub-stepping** scheme. The main `update` function is the public entry point,
 * but it does not advance the simulation by a full frame time. Instead, it
 * accumulates time and performs a series of smaller, stable "sub-steps" until
 * the accumulated time is consumed.
 * - The number of sub-steps is determined by `calculateOptimalSubSteps`, which
 * enforces the **Courant–Friedrichs–Lewy (CFL) condition**. The CFL condition,
 * as described in Bridson's "Fluid Simulation for Computer Graphics", dictates
 * that the time step \f$ \Delta t \f$ must satisfy \f$ \Delta t < \frac{\Delta
 * x}{u_{max}} \f$, where \f$ \Delta x \f$ is the grid spacing and \f$ u_{max}
 * \f$ is the maximum fluid speed. This ensures that a wave does not travel more
 * than one grid cell per step.
 * - The `performSingleSubStep` method contains the core integration logic for
 * one of these small, stable time steps (\f$ \Delta t_{sub} \f$).
 *
 * @subsection Integration Scheme
 * Each sub-step uses an explicit finite difference scheme, based on the methods
 * described by Matthias Müller-Fischer.
 * 1.  **Update Velocities:** The horizontal velocities (`u`, `v`) are updated
 * based on the pressure gradient, which is calculated from the height
 * difference between adjacent cells. The equation is a discrete form of
 * \f$ \frac{\partial \mathbf{v}}{\partial t} = -g \nabla h \f$.
 * 2.  **Update Heights:** The height of each cell is updated based on the net
 * flux of water across its faces. This is a discrete form of the continuity
 * equation, \f$ \frac{\partial h}{\partial t} = -H \nabla \cdot \mathbf{v} \f$,
 * where \f$ H \f$ is the average depth.
 * 3.  **Advection:** The `advectVelocities` method is responsible for
 * transporting the velocity field. It uses a semi-Lagrangian scheme, which
 * traces velocity sample points backward in time through the velocity field and
 * interpolates from the surrounding grid cells to find the new velocity value.
 * This is a common and stable advection technique.
 *
 * @section Boundary Conditions
 * The simulation domain is bounded by solid walls. The boundary conditions are
 * implemented within the interpolation logic in `waterSimulation.cpp`. When a
 * sample is requested from a point outside the grid, the coordinates are
 * clamped to the nearest valid grid cell. This is a **Dirichlet boundary
 * condition** where the fluid state (height and velocity) at the boundary is
 * held constant (at zero velocity), effectively creating a solid, non-slip
 * container for the fluid.
 *
 * @section Rigid Body Coupling
 * The `coupleWithRigidBodies` method implements a full two-way interaction:
 * - **Fluid to Body (Forces):** For each submerged triangle of a rigid body,
 * the simulation calculates hydrostatic buoyancy forces and hydrodynamic drag
 * forces. These forces are derived from the local fluid height and velocity,
 * and are then applied to the rigid body's physics state.
 * - **Body to Fluid (Displacement):** The motion of the rigid body displaces
 * the fluid. The volume displaced by each submerged triangle is used to modify
 * the height of the underlying water grid cells. Likewise, the velocity of the
 * triangle is transferred to the fluid's velocity field, creating waves and
 * disturbances. The `Cdisplacement_SolidsToFluids` and related constants are
 * used to tune the strength and responsiveness of this interaction.
 *
 * @section Multithreading and State Management
 * This simulation is designed to operate within a multithreaded architecture.
 * The `update` method takes two distinct height grids (`newHeightGrid` and
 * `prevHeightGrid`). This is a classic triple-buffering scheme. See the
 * `TripleBufferedChannel` class for more. This architecture allows the main
 * application to run the water simulation in one thread, which calculates the
 * `newHeightGrid` based on the `prevHeightGrid`. Concurrently, a rendering
 * thread can safely read from the (now immutable) `prevHeightGrid` to draw the
 * water surface. This decoupling prevents race conditions and threads blocking.
 *
 * @section Data and Code Sources
 * The algorithms for the heightfield fluid simulation are based on the methods
 * described in:
 * - Matthias Müller-Fischer's 2007 paper, "Real-Time Simulation of Large Bodies
 * of Water with Small Scale Details" (often cited as hffluid.pdf).
 * - Robert Bridson's book, "Fluid Simulation for Computer Graphics", which
 * provides a comprehensive background on grid-based fluid simulation, the CFL
 * condition, and advection schemes.
 *
 * @section Caveats and Assumptions
 * - The heightfield model is fundamentally 2.5D; it cannot represent
 * overturning waves, splashes, caves, or other complex 3D fluid phenomena.
 * - The `velocityComponentDissipationConstant` provides simple numerical
 * damping to ensure stability over long runs, but it is not a physically-based
 * viscosity model.
 */
class WaterSimulation
{
  public:
    /** @brief The floating-point type used for simulation calculations. */
    using Real = float;
    /** @brief A 2D vector type for simulation calculations. */
    using Real2 = linalg::aliases::float2;
    /** @brief A 3D vector type for simulation calculations. */
    using Real3 = linalg::aliases::float3;

    /** @brief The side length of each square grid cell in world units. */
    constexpr static Real cellSize = Physics::WaterSim::cellSize;

    /** @brief The number of grid cells along the z-axis. */
    constexpr static size_t numRows = 100;
    /** @brief The number of grid cells along the x-axis. */
    constexpr static size_t numCols = 80;

    /** @brief The world-space xz-coordinate of the bottom-left corner of the
     * simulation grid. */
    constexpr static Real2 bottomLeftCornerWorldPos_xz{0.025F, 0.025F};
    /** @brief The world-space xz-coordinate of the top-right corner of the
     * simulation grid. */
    constexpr static Real2 topRightCornerWorldPos_xz{
      (static_cast<Real>(numCols - 1) * cellSize) +
        bottomLeftCornerWorldPos_xz.x,
      (static_cast<Real>(numRows - 1) * cellSize) +
        bottomLeftCornerWorldPos_xz.y};

    /**
     * @brief The grid spacing in meters.
     * @details This is equal to `cellSize` and represents the \f$ \Delta x \f$
     * term in the finite difference equations.
     */
    constexpr static Real deltaX = cellSize;

    /**
     * @brief A constant factor to gradually dissipate fluid velocity,
     * simulating viscosity and preventing energy buildup.
     */
    constexpr static Real velocityComponentDissipationConstant = 0.99985F;

    /** @brief A constant vector representing the 'up' direction in the
     * simulation (positive y-axis). */
    constexpr static Real3 upDirection_yHat = {0.0F, 1.0F, 0.0F};

    /** @brief A constant controlling the rate at which fluid disturbances from
     * solids decay. */
    constexpr static Real decayRate_SolidsToFluids = 1.5F;
    static_assert(decayRate_SolidsToFluids > 0.0F);

    /** @brief A coefficient scaling the direct displacement of water by moving
     * solids. */
    constexpr static Real Cdisplacement_SolidsToFluids = 1.0F;
    /** @brief A coefficient scaling the adaptive fluid response to solid
     * interaction. */
    constexpr static Real Cadapt_SolidsToFluids = 0.2F;

    /** @brief The adaptive time step used for each sub-step of the simulation,
     * calculated to maintain stability. */
    inline static Real adaptiveDeltaT = Physics::WaterSim::deltaT;

  private:
    /** @brief The staggered grid data structure holding the fluid's velocity
     * field. */
    StaggeredVelocityGrid<numRows, numCols> velocityGrid{};

    /** @brief The current sub-step being executed within a larger frame update.
     */
    int currentSubStep = 0;
    /** @brief The total number of sub-steps required for the current frame,
     * determined by the CFL condition. */
    int totalSubSteps = 1;
    /** @brief The accumulated simulation time within a frame, used to manage
     * the sub-stepping loop. */
    Real accumulatedTime = 0.0;
    /** @brief A flag indicating if the simulation is currently in the middle of
     * a sub-stepping sequence. */
    bool subStepInProgress = false;

    /**
     * @brief Calculates the optimal number of sub-steps needed to maintain
     * stability for a given frame time.
     * @param heightGrid The current height grid, used to determine maximum wave
     * speed.
     * @return The integer number of sub-steps to perform.
     */
    int calculateOptimalSubSteps(
      const HeightGrid<numRows, numCols>& heightGrid) const;

    /**
     * @brief Performs a single, stable integration step of the fluid
     * simulation.
     * @param newHeightGrid The output height grid for this sub-step.
     * @param prevHeightGrid The input height grid from the previous sub-step.
     * @param subDeltaT The small, adaptive time step for this sub-step.
     */
    void
    performSingleSubStep(HeightGrid<numRows, numCols>& newHeightGrid,
                         const HeightGrid<numRows, numCols>& prevHeightGrid,
                         Real subDeltaT);

    /**
     * @brief Calculates the change in height for a cell based on the divergence
     * of the velocity field.
     * @param i The column index of the cell.
     * @param j The row index of the cell.
     * @param heightGrid The current height grid.
     * @return The change in height, \f$ \Delta h \f$.
     */
    [[nodiscard]] Real calcHeightChangeIntegral(
      size_t i, size_t j, const HeightGrid<numRows, numCols>& heightGrid) const;

    /**
     * @brief Updates the height grid using a semi-implicit integration scheme.
     * @param newHeightGrid The output height grid.
     * @param prevHeightGrid The input height grid.
     */
    void updateHeightsSemiImplicit(
      HeightGrid<numRows, numCols>& newHeightGrid,
      const HeightGrid<numRows, numCols>& prevHeightGrid);

    /**
     * @brief Calculates the change in velocity for a cell face based on the
     * pressure gradient.
     * @param i The column index of the cell.
     * @param j The row index of the cell.
     * @param heightGrid The current height grid, used to calculate pressure.
     * @param accelExt Any external acceleration to apply (e.g., from wind,
     * though unused here).
     * @return A 2D vector containing the change in the u and v velocity
     * components.
     */
    [[nodiscard]] Real2 calcVelocityChangeIntegration(
      size_t i, size_t j, const HeightGrid<numRows, numCols>& heightGrid,
      const Real3& accelExt = {0.0F, 0.0F, 0.0F}) const;

    /**
     * @brief Advects the velocity field using a semi-Lagrangian method.
     * @details This transports the velocity values through the grid according
     * to the flow of the fluid itself, preventing numerical diffusion and
     * ensuring realistic motion.
     */
    void advectVelocities();

    /**
     * @brief Updates the fluid state based on the presence of a submerged rigid
     * body triangle.
     * @param areaOfTriangle The area of the submerged triangle.
     * @param positionOfCentroid The world-space centroid of the triangle.
     * @param velocityOfCentroid The world-space velocity of the triangle's
     * centroid.
     * @param relativeVelocityOfCentroidWRTFluid The velocity of the triangle
     * relative to the fluid.
     * @param normalOfCentroid The world-space normal of the triangle.
     * @param heights The current height grid, which will be modified by this
     * function.
     */
    void updateFluidWithTriangle(
      Real areaOfTriangle, const Real3& positionOfCentroid,
      const Real3& velocityOfCentroid,
      const Real3& relativeVelocityOfCentroidWRTFluid,
      const Real3& normalOfCentroid,
      HeightGrid<WaterSimulation::numRows, WaterSimulation::numCols>& heights);

    /**
     * @brief Computes the fluid dynamic forces (drag and buoyancy) acting on a
     * submerged triangle.
     * @param subTriangle The submerged triangle patch to calculate forces for.
     * @param triangleVelocity The velocity of the triangle.
     * @param heights The current height grid of the fluid.
     * @return A 3D vector representing the total force exerted by the fluid on
     * the triangle.
     */
    [[nodiscard]] Real3 computeFluidForceOnTriangle(
      const SubTriangle& subTriangle, const Real3& triangleVelocity,
      const HeightGrid<numRows, numCols>& heights) const;

    /**
     * @brief Gets the interpolated fluid velocity at an arbitrary world
     * position.
     * @param worldPos The xz-position in world space to sample the velocity.
     * @return A 3D vector representing the fluid velocity at that point
     * (y-component will be zero).
     */
    [[nodiscard]] Real3 getFluidVelocityAtPosition(const Real3& worldPos) const;

    /** @brief A counter to trigger stability checks periodically. */
    mutable int stabilityCheckCounter = 0;
    /** @brief The total energy of the system from the previous stability check,
     * used to monitor for explosions. */
    mutable Real previousTotalEnergy = 0.0F;
    /**
     * @brief Performs a check for volume conservation and energy stability.
     * @details This is a debugging and stability-enhancing routine that can
     * detect if the simulation is becoming unstable (e.g., "exploding" due to
     * numerical error).
     * @param heightGrid The current height grid to analyse.
     */
    void performStabilityCheck(const HeightGrid<numRows, numCols>& heightGrid);

  public:
    /**
     * @brief Constructs the water simulation.
     */
    WaterSimulation();

    /** @brief Deleted copy constructor. */
    WaterSimulation(const WaterSimulation&) = delete;
    /** @brief Deleted move constructor. */
    WaterSimulation(WaterSimulation&&) = delete;
    /** @brief Deleted copy assignment operator. */
    WaterSimulation& operator=(const WaterSimulation&) = delete;
    /** @brief Deleted move assignment operator. */
    WaterSimulation& operator=(WaterSimulation&&) = delete;

    /**
     * @brief Sets the initial state of the water surface.
     * @param heightGrid The height grid to be initialized, typically with a
     * disturbance like a sine wave.
     */
    static void
    setInitConditions(HeightGrid<WaterSimulation::numRows,
                                 WaterSimulation::numCols>& heightGrid);

    /**
     * @brief Advances the fluid simulation by one time step, performing
     * multiple sub-steps as needed for stability.
     * @param newHeightGrid The output height grid for the next frame.
     * @param prevHeightGrid The input height grid from the previous frame.
     */
    void update(HeightGrid<numRows, numCols>& newHeightGrid,
                const HeightGrid<numRows, numCols>& prevHeightGrid);

    /**
     * @brief Checks if a given 3D world position is currently under the water
     * surface.
     * @param pos The world-space position to check.
     * @param heights The current height grid of the water.
     * @return `true` if the position is submerged, `false` otherwise.
     */
    [[nodiscard]] static bool
    isPositionInWater(const Real3& pos,
                      const HeightGrid<WaterSimulation::numRows,
                                       WaterSimulation::numCols>& heights);

    /**
     * @brief Manages the two-way interaction between the fluid and a set of
     * rigid bodies.
     * @param rigidBodies The list of all dynamic rigid bodies in the scene.
     * @param heights The current height grid, which will be modified by the
     * bodies' motion.
     */
    void coupleWithRigidBodies(
      std::vector<RigidBodyData>& rigidBodies,
      HeightGrid<WaterSimulation::numRows, WaterSimulation::numCols>& heights);
};
