![EdynLogo](https://user-images.githubusercontent.com/762769/211650462-3ad6dab2-5e47-4b62-993c-ac7fc7650cde.svg)

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

After cloning the repo using [Git Bash](https://git-scm.com/downloads/win) and assuming [Conan 2.x](https://conan.io/) is installed, enter the following commands:
```
$ conan install conanfile.txt -of=build
$ cd build
$ cmake .. -G "Visual Studio 16 2019" -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake
```

The _EdynTestbed.sln_ solution should be in the _build_ directory. It's important to set the working directory by going to the `EdynTestbed` target properties (`Alt Enter`) and under `Configuration Properties > Debugging > Working Directory` assign the bgfx examples runtime directory, such as `$(ProjectDir)..\..\bgfx\examples\runtime`.

## Sounds

Some samples include sound effects, played via [SoLoud](https://github.com/jarikomppa/soloud). To enable sounds, set the CMake option `EDYN_SOUND_ENABLED` to true. You will have to link `SoLoud` and `SDL2`.

It uses these sounds from freesound:
- "Billiard balls single hit-dry.wav" by juskiddink (https://freesound.org/people/juskiddink/sounds/108615/) licensed under CCBYNC 3.0
- "big thud.wav" by Reitanna (https://freesound.org/people/Reitanna/sounds/332661/) licensed under CCBY 3.0

## Networked physics

To build the networked physics samples, set the CMake option `EDYN_BUILD_NETWORKING_EXAMPLE` to true and to build the server applications, set `EDYN_BUILD_SERVER` to true. The networked physics samples and the servers need the [ENet](https://github.com/lsalzman/enet) library.

The servers are separate applications. Each distinct networking sample has a server associated with it and the server must be running before the sample is selected in the sample browser so it can connect to server. If the server is running in a different machine, it's necessary to edit the host name is the calls to `ExampleBasicNetworking::connectToServer`.

# Running it

Press `P` to pause/unpause the simulation. Press `L` to step the simulation when paused.

https://user-images.githubusercontent.com/762769/148439511-9dad8f36-182c-43e7-af91-bdd43f430965.mp4


https://user-images.githubusercontent.com/762769/161366613-565b2dff-1d91-4fa1-8691-dbb5ae8abbff.mp4
