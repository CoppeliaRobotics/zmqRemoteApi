# ZeroMQ Remote API for CoppeliaSim

The ZeroMQ Remote API requires the [ZeroMQ plugin](https://github.com/CoppeliaRobotics/simExtZeroMQ).

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

### Usage

Connect a `REQ` socket to the endpoint (by default the API server will listen to `tcp://*:23000`), send a message (see [request](#request) below), and read the response (see [response](#response) below). The request and response can be serialized to [JSON](https://www.json.org) or [CBOR](https://cbor.io). The response will be serialized using the same serialization format used in the request.

Example implementation for some languages are available in the [clients](tree/master/clients) directory.

### Request

A request is an object with fields:
- `func` (string) the function name to call;
- `args` (array) the arguments to the function;

Example:

```json
{
    "func": "sim.getObjectHandle",
    "args": ["Floor"]
}
```

### Response

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
    "error": "Object does not exist. (in function 'sim.getObjectHandle')"
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
sim = client.getobject('sim')

# call API function:
h = sim.getObjectHandle('Floor')
print(h)
```

#### Python asyncio client

Normal `asyncio` principles apply. All methods are async.

```python
async def main():
    async with RemoteAPIClient() as client:
        sim = await client.getobject('sim')
        h = await sim.getObjectHandle('Floor')
        print(h)

asyncio.run(main())
```

If performing many commands in one shot, and results will be used later, consider using `asyncio.gather` for improved throughput. E.g. getting the handles of 100 objects:

```python
    handles = await asyncio.gather(*[sim.getObjectHandle(f'Object{i+1}') for i in range(100)])
```