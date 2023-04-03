# CoppeliaSim zmqRemoteApi Python client

Python client for the zmqRemoteApi.

## Installing:

```sh
python3 -m pip install coppeliasim-zmqremoteapi-client
```

## Usage

```python
from coppeliasim_zmqremoteapi_client import *

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

There is also an `asyncio` version of the client. Normal `asyncio` principles apply, and all methods are async:

```python
from coppeliasim_zmqremoteapi_client.asyncio import *

client = RemoteAPIClient()

async def main():
    async with RemoteAPIClient() as client:
        sim = await client.getObject('sim')
        h = await sim.getObject('/Floor')
        print(h)

asyncio.run(main())
```

on Windows, if it doesn't work properly, before calling `asyncio.run(...)` call:

```python
asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
```

A note about performance of sequential requests: if performing many commands in one shot, and results will be used later, consider using `asyncio.gather` for improved throughput.

E.g. getting the handles of 100 objects:

```python
handles = await asyncio.gather(*[sim.getObject(f'/Object{i+1}') for i in range(100)])
```

Also check out the examples in [`clients/python`](https://github.com/CoppeliaRobotics/zmqRemoteApi/blob/master/clients/python).
