![EdynLogo](https://xissburg.com/images/EdynLogo.svg)

# Edyn Testbed

Examples for the [Edyn](https://github.com/xissburg/edyn) physics engine.

Dependencies:
- [EnTT](https://github.com/skypjack/entt) (installed via [Conan](https://conan.io/))
- [bgfx](https://github.com/bkaradzic/bgfx), [bx](https://github.com/bkaradzic/bx) and [bimg](https://github.com/bkaradzic/bimg) are assumed to be cloned inside the same directory as _edyn-testbed_. This project is built using _bgfx_'es example framework thus it needs to be ran from the `bgfx/examples/runtime` directory to correctly load shaders and other files. In other words, `bgfx/examples/runtime` must be the _working directory_.
- [Git LFS](https://git-lfs.github.com/) is needed to download the resource files. You might have to run `git lfs pull` to bring in the files.

# Building

In the terminal, go into the _edyn-testbed_ directory and do:

```
$ mkdir build
$ cd build
$ conan install ../conanfile.txt
$ cmake ..
$ make
```

You might have to tell CMake where to find the _bgfx_ and _Edyn_ libraries. For _bgfx_, just set `Bgfx_LIBRARY_DIR` to the directory where the _bgfx_ libraries are located and CMake should find and assign all of them.

## Windows and Visual Studio 2019

After running `cmake ..`, the _Edyn.sln_ solution should be in the _build_ directory. It's important to set the working directory by going to the `EdynTestbed` target properties (`Alt Enter`) and under `Configuration Properties > Debugging > Working Directory` assign the bgfx examples runtime directory, such as `$(ProjectDir)..\..\bgfx\examples\runtime`.

## Sounds

Some samples include sound effects, played via [SoLoud](https://github.com/jarikomppa/soloud). To enable sounds, set the CMake option `EDYN_SOUND_ENABLED` to true. You will have to link `SoLoud` and `SDL2`.

It uses these sounds from freesound:
- "Billiard balls single hit-dry.wav" by juskiddink (https://freesound.org/people/juskiddink/sounds/108615/) licensed under CCBYNC 3.0
- "big thud.wav" by Reitanna (https://freesound.org/people/Reitanna/sounds/332661/) licensed under CCBY 3.0

# Running it

Press `P` to pause/unpause the simulation. Press `L` to step the simulation when paused.

https://user-images.githubusercontent.com/762769/148439511-9dad8f36-182c-43e7-af91-bdd43f430965.mp4

