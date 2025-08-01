# Brief Description

This project focuses on simulating the interaction between concave meshes (such as a bottle) and a body of water in real-time.
It features a position-based 3D physics engine written from scratch, which supports arbitrary concave meshes by computing their inertia tensor.
To simulate the water, we implement a height field fluid simulation with adaptive time-stepping, and carefully couple forces between it and the physics engine.
To ensure that our simulation updates do not block our rendering and vice-versa, the project features robust multithreading.
The entire scene is then rasterised by OpenGL (we use version 3.3 for compatibility).

For more details, kindly see the report.

# How to Run This?

CMake gave us a lot of problems throughout development.
If something does not work, please contact us!

## Installing Dependencies

When the repo is first pulled down, you must pull down the relevant git submodules of dependency libraries.
To do so, execute,
```sh
git submodule update --init --recursive
```

## Compilation

### Generating Build Files

Skip the first step if you have already built the project once and have not added/removed source or header files nor edited cmake build files.
In this instance, you don't need new build files.

If you have not built the project yet, or if you add/remove source or header files, first run,
```sh
# for windows
cmake --preset windows-debug   # for debug mode
cmake --preset windows-release # for release mode

# for everyone else
cmake --preset debug   # for debug mode
cmake --preset release # for release mode
```
This generates the actual build files for your compilation setup.

***IMPORTANT NOTE:***
Your default compiler may not support C++20 fully!
This can lead to strange circumstances where, e.g, you cannot find `std::format` during compilation.
If you are on the Linux student server, then this is most likely the case.
Please supply a C++ compiler that can support C++20 via the command below
```sh
cmake -DCMAKE_CXX_COMPILER=/path/to/compiler --preset <your_preset>
```
### Building

Then to actually build the project run,
```sh
# for windows
cmake --build --preset windows-debug -j 8   # for debug build
cmake --build --preset windows-release -j 8 # for release build

# for everyone else
cmake --build --preset debug -j 8   # for debug build
cmake --build --preset release -j 8 # for release build
```
## Running

Once built run the `CS488` executable to launch the program.

For Windows, the executable should be `build/windows-debug/Debug/CS488.exe` or `build/windows-release/Release/CS488.exe` or maybe even in `build/windows-debug/CS488.exe` or `build/windows-release/CS488.exe`.
Please make sure to run from the build directory to ensure assets/shaders are found correctly.

Otherwise, the executable should be `build/debug/CS488` or `build/release/CS488`.

# Implementation

## Sources

### Models

We credit the following models to their respective sources:

- The model of the goose was taken from a source online: Goose by Poly by Google [CC-BY](https://creativecommons.org/licenses/by/3.0/) via [Poly Pizza](https://poly.pizza/m/9wn3If7Qgb4). We modified the `.mtl` file to adapt it to our renderer.

- Angular velocity
    - https://www.euclideanspace.com/physics/kinematics/angularvelocity/
    - https://www.researchgate.net/publication/46422891_Robust_rotational-velocity-Verlet_integration_methods

# Objectives

## Main Objectives

All of the following were achieved:

1. Replace CPU rasterisation with OpenGL rasterisation pipeline. Write vertex and fragment shaders to implement MVP matrix, diffuse and specular texture maps, and Phong shading.
2. Model the objects required for the scene, such as the bottle.
3. Simulate 2D shallow water as a height field by integrating height and velocity as described in the technical outline.
4. Solve advection equations and add boundary conditions to simulate the waves accurately as described in the technical outline.
5. Implement a geometry shader to rasterise the surface of the water such that it appears continuous as described in the technical outline.
6. Implement rigid-body physics for the bottle using Verlet integration. Handle collisions with the pool and other objects with bounding boxes as described in the technical outline.
7. Simulate change in water levels and waves when objects collide as described in the technical outline.
8. Implement fluid-body interaction (buoyancy and drag) as described in the technical outline.

