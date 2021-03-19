![EdynLogo](https://xissburg.com/images/EdynLogo.svg)

# Edyn Testbed

Examples for the [Edyn](https://github.com/xissburg/edyn) physics engine.

Dependencies:
- [EnTT](https://github.com/skypjack/entt) (installed via [Conan](https://conan.io/))
- [bgfx](https://github.com/bkaradzic/bgfx), [bx](https://github.com/bkaradzic/bx) and [bimg](https://github.com/bkaradzic/bimg) are assumed to be cloned inside the same directory as _edyn-testbed_. This project is built using _bgfx_'es example framework thus it needs to be ran from the `bgfx/examples/runtime` directory to correctly load shaders and other files. In other words, `bgfx/examples/runtime` must be the _working directory_.

In the _edyn-testbed_ directory:

```
$ mkdir build
$ cd build
$ conan install ../conanfile.txt
$ cmake ..
$ make
```

Press `P` to pause/unpause the simulation. Press `L` to step the simulation when paused.

![Screenshot from 2020-08-01 16-39-21](https://user-images.githubusercontent.com/762769/111734026-3947e280-8847-11eb-9ee5-a284295af185.png)
