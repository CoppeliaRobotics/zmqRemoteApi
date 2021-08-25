"""CoppeliaSim's Remote API client."""

import asyncio

import cbor

import zmq
import zmq.asyncio


if sys.platform == 'win32' and sys.version_info >= (3, 8, 0):
    if isinstance(asyncio.get_event_loop_policy(), asyncio.windows_events.WindowsProactorEventLoopPolicy):
        print('''

    WARNING: on Windows and Python 3.8+, `asyncio` might not work properly; in case, add the following before `asyncio.run(...)`:

    if sys.platform == 'win32' and sys.version_info >= (3, 8, 0):
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

''')

class RemoteAPIClient:
    """Client to connect to CoppeliaSim's ZMQ Remote API."""

    def __init__(self, host='localhost', port=23000, *, verbose=False):
        """Create client and connect to the ZMQ Remote API server."""
        self.verbose = verbose
        self.host, self.port = host, port
        # multiple sockets will be created for multiple concurrent requests, as needed
        self.sockets = []
        self.sim = None

    async def __aenter__(self):
        """Add one socket to the pool."""
        self.context = zmq.asyncio.Context()
        self._add_socket()
        return self

    async def __aexit__(self, *excinfo):
        """Disconnect and destroy client."""
        for socket in self.sockets:
            socket.close()
        self.context.term()

    def _add_socket(self):
        socket = self.context.socket(zmq.REQ)
        socket.connect(f'tcp://{self.host}:{self.port}')
        self.sockets.append(socket)
        return socket

    async def call(self, func, args, verbose=None):
        """Call function with specified arguments."""
        def deepmapitem(fn, d):
            if isinstance(d, (list, tuple)):
                return type(d)(deepmapitem(fn, x) for x in d)
            elif isinstance(d, dict):
                return {k: deepmapitem(fn, v) for k, v in map(lambda t: fn(*t), d.items())}
            else:
                return d
        if verbose is None:
            verbose = self.verbose
        req = {'func': func, 'args': args}
        if verbose:
            print(req)
        if not self.sockets:
            socket = self._add_socket()
        socket = self.sockets.pop()
        await socket.send(cbor.dumps(req))
        resp = cbor.loads(await socket.recv())
        self.sockets.append(socket)
        resp = deepmapitem(lambda k, v: (k.decode('utf8'), v), resp)
        if verbose:
            print(resp)
        if not resp.get('success', False):
            raise Exception(resp.get('error'))
        ret = resp['ret']
        if len(ret) == 1:
            return ret[0]
        if len(ret) > 1:
            return tuple(ret)

    async def getobject(self, name, _info=None):
        """Retrieve remote object from server."""
        ret = type(name, (), {})
        if not _info:
            _info = await self.call('zmqRemoteApi.info', [name])
        for k, v in _info.items():
            if not isinstance(v, dict):
                raise ValueError('found nondict')
            if len(v) == 1 and 'func' in v:
                setattr(ret, k, lambda *a, func=f'{name}.{k}': self.call(func, a))
            elif len(v) == 1 and 'const' in v:
                setattr(ret, k, v['const'])
            else:
                setattr(ret, k, self.getobject(f'{name}.{k}', _info=v))
        return ret

    async def call_addon(self, func, *args):
        if self.sim is None:
            self.sim = await sef.getobject('sim')
        return self.sim.callScriptFunction(f'{func}@ZMQ remote API', self.sim.scripttype_addonscript, *args)

    async def setsynchronous(self, enable=True):
        return self.call_addon('setSynchronous', enable)

    async def step(self):
        return self.call_addon('step')


async def main():
    """Test basic usage."""
    async with RemoteAPIClient() as client:
        sim = await client.getobject('sim')
        print(await sim.getObjectHandle('Floor'))
        print(await sim.unpackTable(await sim.packTable({'a': 1, 'b': 2})))
        handles = await asyncio.gather(*[sim.createDummy(0.01, 12 * [0]) for _ in range(50)])
        await asyncio.gather(*[sim.setObjectPosition(h, -1, [0.01 * i, 0.01 * i, 0.01 * i]) for i, h in enumerate(handles)])
        await asyncio.sleep(10)
        await asyncio.gather(*[sim.removeObject(h) for h in handles])


if __name__ in ('__main__',):
    asyncio.run(main())
