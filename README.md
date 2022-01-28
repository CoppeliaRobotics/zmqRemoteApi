# ZeroMQ Remote API for CoppeliaSim

The ZeroMQ Remote API requires the [ZeroMQ plugin](https://github.com/CoppeliaRobotics/simExtZeroMQ).

### Table of contents

 - [Compiling](#compiling)
 - [Protocol](#protocol)
     - [Request](#request)
     - [Response](#response)
 - [Clients](#clients)
     - [Python](#python-client)
     - [Python-asyncio](#python-asyncio-client)
     - [C++](#c-client)
     - [Java](#java-client)
     - [Octave](#octave-client)
     - [MATLAB](#matlab-client)
     - [HTML/JavaScript](#htmljavascript-web-browser-client)


### Compiling

1. Install required packages for [libPlugin](https://github.com/CoppeliaRobotics/libPlugin): see libPlugin's [README](external/libPlugin/README.md)
2. Checkout and compile
```text
$ git clone --recursive https://github.com/CoppeliaRobotics/zmqRemoteApi
$ mkdir zmqRemoteApi/build
$ cd zmqRemoteApi/build
$ cmake ..
$ cmake --build .
$ cmake --install .
```

### Protocol

Connect a [`REQ`](https://zeromq.org/socket-api/#req-socket) socket to the endpoint (by default the ZMQ remote API server will listen to `tcp://*:23000`), send a message (see [request](#request) below), and read the response (see [response](#response) below). The request and response can be serialized to [JSON](https://www.json.org) or [CBOR](https://cbor.io). The response will be serialized using the same serialization format used in the request.

Example implementation for some languages are available in the [clients](tree/master/clients) directory.

#### Request

A request is an object with fields:
- `func` (string) the function name to call;
- `args` (array) the arguments to the function;

Example:

```json
{
    "func": "sim.getObject",
    "args": ["/Floor"]
}
```

#### Response

A response is an object with fields:
- `success` (boolean) `true` if the call succeeded, in which case the `ret` field will be set, or `false` if the call failed, in which case the `error` field will be set;
- `ret` (array) the return values of the function;
- `error` (string) the error message;

Example:

```json
{
    "success": true,
    "ret": [37]
}
```

In case of error, the exception message will be present:

```json
{
    "success": false,
    "error": "Object does not exist. (in function 'sim.getObject')"
}
```

### Clients

#### Python client

(Make sure to have directory `zmqRemoteApi` (from `clients/python`) somewhere in `sys.path`)

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

    auto ret = client.call("sim.getObject", {"/Floor"});
    int handle = ret[0];

    return 0;
}
```

Check out the examples in [`clients/cpp`](clients/cpp).

#### Java client

```java
RemoteAPIClient client = new RemoteAPIClient();
JSONArray ret = client.call("sim.getObject", "/Floor");
int handle = ret.get(0);
```

Check out the examples in [`clients/java`](clients/java).

#### Octave client

```octave
client = RemoteAPIClient();
ret = client.call('sim.getObject', {'/Floor'})
```

Check out the examples in [`clients/octave`](clients/octave).

#### MATLAB client

```matlab
client = RemoteAPIClient();
ret = client.call('sim.getObject', {'/Floor'})
```

Check out the examples in [`clients/matlab`](clients/matlab).

#### HTML/JavaScript (web browser) client

ZeroMQ is not available for web browsers.

See [wsRemoteApi](https://github.com/CoppeliaRobotics/wsRemoteApi) instead.
