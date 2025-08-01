#pragma once

#include "util/math.hpp"

#include <linalg.h>

namespace Physics
{

constexpr float gravitationalAccelerationMagnitude = 9.80665F;

const linalg::aliases::float3 gravitationalAcceleration{
  0.0F, -gravitationalAccelerationMagnitude, 0.0F};

namespace WaterSim
{

constexpr static float cellSize = 0.05F;

// TODO: the paper suggests a better maxDepth in section 2.1.5
// TODO: should we/can we lower the base depth of 1.0 further?
constexpr static float maxDepth = 1.5F;

// Bound suggested in Fluid Simulation for Computer Graphics by R. Bridson
// in section 12.3.
// Incorrect computation. See WaterSimulation::calculateOptimalSubsteps instead
// constexpr static float deltaTBoundAbove =
//   cellSize / CS488Math::sqrt(gravitationalAccelerationMagnitude * maxDepth);

// to ensure fixed deltaT is bounded above by the...bound given above, we
// use a fractional value of the bound (0.2 is suggested by Fluid
// Simulation for Computer Graphics by R. Bridson in section 12.3).
// constexpr static float deltaT = 0.0002F;
// constexpr static float deltaT = deltaTBoundAbove * 0.005F;
// Other bounds on deltaT:
// - t_n + deltaT < t_frame  [SCG by R.B. section 2.3]
// -

constexpr static float deltaT = 0.0002F;

} // namespace WaterSim

namespace RigidBody
{

// A *much* bigger deltaT for physics
// constexpr static float deltaT = deltaTBoundAbove * 0.02F;
constexpr static float deltaT = WaterSim::deltaT;

} // namespace RigidBody

} // namespace Physics
