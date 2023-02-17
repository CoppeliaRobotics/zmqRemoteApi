# ZMQ Remote API for CoppeliaSim

The ZMQ Remote API requires the [ZMQ plugin](https://github.com/CoppeliaRobotics/simExtZMQ).

### Table of contents

 - [Compiling](#compiling)
 - [Clients](#clients)
     - [Python](#python-client)
     - [Python-asyncio](#python-asyncio-client)
     - [C++](#c-client)
     - [Java](#java-client)
     - [Octave](#octave-client)
     - [MATLAB](#matlab-client)
     - [Rust](#rust-client)
     - [HTML/JavaScript](#htmljavascript-web-browser-client)


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

#### Python client

(Make sure to have directory `zmqRemoteApi/clients/python` in `sys.path`/`$PYTHONPATH`)

```python
import zmqRemoteApi

# create a client to connect to zmqRemoteApi server:
# (creation arguments can specify different host/port,
# defaults are host='localhost', port=23000)
client = RemoteAPIClient()

# get a remote object:
sim = client.getObject('sim')

# call API function:
h = sim.getObject('/Floor')
print(h)
```

Check out the examples in [`clients/python`](clients/python).

#### Python asyncio client

Normal `asyncio` principles apply. All methods are async.

```python
async def main():
    async with RemoteAPIClient() as client:
        sim = await client.getObject('sim')
        h = await sim.getObject('/Floor')
        print(h)

asyncio.run(main())
```

Note: on Windows and Python 3.8+, `asyncio` might not work properly; in case, just add the following before `asyncio.run`:

```python
if sys.platform == 'win32' and sys.version_info >= (3, 8, 0):
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
```

If performing many commands in one shot, and results will be used later, consider using `asyncio.gather` for improved throughput. E.g. getting the handles of 100 objects:

```python
    handles = await asyncio.gather(*[sim.getObject(f'/Object{i+1}') for i in range(100)])
```

Check out the examples in [`clients/python`](clients/python).

#### C++ client

```cpp
#include "RemoteAPIClient.h"
#include <iostream>

int main()
{
    RemoteAPIClient client;
    auto sim = client.getObject().sim();

    int handle = sim.getObject("/Floor");

    return 0;
}
```

Check out the examples in [`clients/cpp`](clients/cpp).

#### Java client

The Java client is still experimental.

See [here](clients/java).

#### Octave client

```octave
client = RemoteAPIClient();
sim = client.getObject('sim');
handle = sim.getObject('/Floor')
```

Check out the examples in [`clients/octave`](clients/octave).

#### MATLAB client

```matlab
client = RemoteAPIClient();
sim = client.getObject('sim');
handle = sim.getObject('/Floor')
```

Check out the examples in [`clients/matlab`](clients/matlab).

#### Rust client

The Rust client is made available by [Samuel Cavalcanti](https://github.com/samuel-cavalcanti).

See [here](https://github.com/samuel-cavalcanti/rust_zmqRemoteApi).

#### HTML/JavaScript (web browser) client

ZeroMQ is not available for web browsers.

See [wsRemoteApi](https://github.com/CoppeliaRobotics/wsRemoteApi) instead.
