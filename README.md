# Installing Dependencies

When the repo is first pulled down, you must pull down the relevant git submodules of dependency libraries.
To do so, execute,
```sh
git submodule update --init --recursive
```

# Compilation

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

Skip the first step if you have already built the project once and have not added/removed source or header files nor edited cmake build files.
In this instance, you don't need new build files.

Then to actually build the project run,
```sh
# for windows
cmake --build --preset windows-debug -j 8   # for debug build
cmake --build --preset windows-release -j 8 # for release build

# for everyone else
cmake --build --preset debug -j 8   # for debug build
cmake --build --preset release -j 8 # for release build
```

# Run

Once built run the `CS488` executable to launch the program.

For Windows, the executable should be `build/windows-debug/Debug/CS488.exe` or `build/windows-release/Release/CS488.exe`.
Please make sure to run from the build directory to ensure assets/shaders are found correctly.

Otherwise, the executable should be `build/debug/CS488` or `build/release/CS488`.

# TODO

## sim

- [ ] Boundary conditions need to handle corners!! Right now corner just gets set to neighbouring cell in the boundary instead of diagonal cell. Sometimes this is done in the wrong order and you get weirdness in the corners.
- [ ] add some sort of dampening to the waves so that they don't just go on forever
    - Maybe a minimum wave size?
- [ ] make water surface look like actual actual water
    - some sort of interpolation to make the triangles less sharp. 
        Maybe instead of rendering the heightfield's heights as the vertices, interpolate a vertex in the middle of the heights and render that?
    - better texture and lighting ofc
    - better transparency

## performance

- [ ] Move Projection View matrix computation to CPU? See vertex shader for comment explaining more
- [ ] Code sets some uniforms, such as texture uniform each draw loop. Can this lessened?

## aesthetics

- [ ] Gamma correction

## meta
- [ ] Clean up TODOs around the codebase
- [ ] Need better abstraction for world objects.
        So currently point Lights are not a `Model`. 
        Makes sense since `Model` is more as a container around meshes and materials that form an object.
        Point Lights should have a `Model` and not be one.
        The problem is `Model` stores its model matrix, and point lights store their own position separately.
        So then we have data duplication.
        So ideally we need a different abstraction to hold the model matrix? Unsure what form this would take.
- [ ] Need a better abstraction for the draw loop. 
        Currently we write an ugly function in renderer. 
        This will get painful the more objects we render at once sigh.
- [ ] Need better abstraction for shaders.
        Checking uniforms is painful to write code for and makes swapping shaders really hard.
        Currently it is more painful than it should be to render something without light shader for example.

## Done

- [x] Blinn-Phong lighting
- [x] Better error reporting. Currently program just crashes outside of debugger. Need async error reporting thread tbh.
- [x] Code does not support non-textured items. Should it?

# TA Questions
- How to move backend work to a different thread. Making a different thread rotate things feasible?
    - Try without first to see performance.
- Making objectives easier
    - One simplification is making the pool limitless (no more boundary collision)
    - Move 8 off the proposal
- How to Gamma correct?
    - Check wikipedia. Not required.
