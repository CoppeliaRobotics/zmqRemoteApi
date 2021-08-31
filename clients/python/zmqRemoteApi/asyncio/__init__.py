"""CoppeliaSim's Remote API client."""

import asyncio
import sys

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


def b64(b):
    import base64
    return base64.b64encode(b).decode('ascii')


class RemoteAPIClient:
    """Client to connect to CoppeliaSim's ZMQ Remote API."""

    def __init__(self, host='localhost', port=23000, cntport=None, *, verbose=False):
        """Create client and connect to the ZMQ Remote API server."""
        self.verbose = verbose
        self.host, self.port, self.cntport = host, port, cntport or port + 1
        self.cntsocket = None
        # multiple sockets will be created for multiple concurrent requests, as needed
        self.sockets = []
        self.sim = None

    async def __aenter__(self):
        """Add one socket to the pool."""
        self.context = zmq.asyncio.Context()
        self.cntsocket = self.context.socket(zmq.SUB)
        self.cntsocket.setsockopt(zmq.SUBSCRIBE, b'')
        self.cntsocket.setsockopt(zmq.CONFLATE, 1)
        self.cntsocket.connect(f'tcp://{self.host}:{self.cntport}')
        self._add_socket()
        return self

    async def __aexit__(self, *excinfo):
        """Disconnect and destroy client."""
        for socket in self.sockets:
            socket.close()
        self.cntsocket.close()
        self.context.term()

    def _add_socket(self):
        socket = self.context.socket(zmq.REQ)
        socket.connect(f'tcp://{self.host}:{self.port}')
        self.sockets.append(socket)
        return socket

    async def call(self, func, args, *, verbose=None):
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
            print('Sending:', req)
        if not self.sockets:
            socket = self._add_socket()
        socket = self.sockets.pop()
        rawReq = cbor.dumps(req)
        if verbose:
            print(f'Sending raw len={len(rawReq)}, base64={b64(rawReq)}')
        await socket.send(rawReq)
        rawResp = await socket.recv()
        if verbose:
            print(f'Received raw len={len(rawResp)}, base64={b64(rawResp)}')
        resp = cbor.loads(rawResp)
        self.sockets.append(socket)
        resp = deepmapitem(lambda k, v: (k.decode('utf8'), v), resp)
        if verbose:
            print('Received:', resp)
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
            self.sim = await self.getobject('sim')
        return await self.sim.callScriptFunction(f'{func}@ZMQ remote API', self.sim.scripttype_addonscript, *args)

    async def setsynchronous(self, enable=True):
        return await self.call_addon('setSynchronous', enable)

    async def step(self, *, wait=True):
        async def hasnewstepcount():
            poller = zmq.asyncio.Poller()
            poller.register(self.cntsocket, zmq.POLLIN)
            socks = dict(await poller.poll(0))
            b = self.cntsocket in socks and socks[self.cntsocket] == zmq.POLLIN
            return b

        async def getstepcount():
            import struct
            return struct.unpack('i', await self.cntsocket.recv())[0]

        if wait and await hasnewstepcount():
            await getstepcount()
        await self.call_addon('step')
        if wait:
            await getstepcount()


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


__all__ = ['RemoteAPIClient']
