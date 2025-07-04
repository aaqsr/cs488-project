# Compilation

If you have not built the project yet, or if you add/remove source or header files, first run,
```sh
cmake --preset debug   # for debug mode
cmake --preset release # for release mode
```
Skip the first step if you have already built the project once and have not added/removed source or header files.

Then to actually build the project run,
```sh
cmake --build --preset debug --parellel   # for debug build
cmake --build --preset release --parallel # for debug build ```

This will compile an executable in `./build/debug` or `./build/release`.
To execute the program, run
```sh
./build/debug/CS488   # for debug mode
./build/release/CS488 # for release mode
```

# TODO

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
- [ ] Clean up TODOs around the codebase
- [ ] Move Projection View matrix computation to CPU? See vertex shader for comment explaining more
- [ ] Code sets some uniforms, such as texture uniform each draw loop. Can this lessened?
- [ ] Gamma correction

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
