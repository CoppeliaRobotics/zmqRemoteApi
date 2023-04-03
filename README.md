# ZMQ Remote API for CoppeliaSim

The ZMQ Remote API requires the [ZMQ plugin](https://github.com/CoppeliaRobotics/simExtZMQ).

### Compiling

1. Install required packages for simStubsGen: see simStubsGen's [README](https://github.com/CoppeliaRobotics/include/blob/master/simStubsGen/README.md)
2. Checkout and compile
```sh
$ git clone --recursive https://github.com/CoppeliaRobotics/zmqRemoteApi
$ cd zmqRemoteApi
$ git checkout coppeliasim-v4.5.0-rev0
$ mkdir -p build && cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ..
$ cmake --build .
$ cmake --install .
```

NOTE: replace `coppeliasim-v4.5.0-rev0` with the actual CoppeliaSim version you have.

### Clients

 - [Python](clients/python)
 - [C++](clients/cpp)
 - [Java](clients/java)
 - [Octave](clients/octave)
 - [MATLAB](clients/matlab)
 - HTML/JavaScript: not available for browsers; use [wsRemoteApi](https://github.com/CoppeliaRobotics/wsRemoteApi) instead.

Third party:

 - Rust: [samuel-cavalcanti/rust_zmqRemoteApi](https://github.com/samuel-cavalcanti/rust_zmqRemoteApi)
