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

- [ ] Better error reporting. Currently program just crashes outside of debugger. Need async error reporting thread tbh.
- [ ] Clean up TODOs around the codebase
- [ ] Code does not support non-textured items. Should it?
