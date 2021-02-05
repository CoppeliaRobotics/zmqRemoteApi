import json
import cbor
import asyncio
import zmq
import zmq.asyncio

def deepmapitem(fn, d):
    if isinstance(d, (list, tuple)):
        return type(d)(deepmapitem(fn, x) for x in d)
    elif isinstance(d, dict):
        return {k: deepmapitem(fn, v) for k, v in map(lambda t: fn(*t), d.items())}
    else:
        return d

class RemoteAPIClient:
    def __init__(self, host='localhost', port=23000, *, verbose=False):
        self.context = zmq.asyncio.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.verbose = verbose
        self.socket.connect('tcp://{}:{}'.format(host, port))
    def __del__(self):
        self.socket.close()
        self.context.term()
    async def call(self, func, args, verbose=None):
        if verbose is None: verbose = self.verbose
        req = {'func': func, 'args': args}
        if verbose: print(req)
        await self.socket.send(cbor.dumps(req))
        resp = cbor.loads(await self.socket.recv())
        resp = deepmapitem(lambda k, v: (k.decode('utf8'), v), resp)
        if verbose: print(resp)
        if not resp.get('success', False): raise Exception(resp.get('error'))
        ret = resp['ret']
        if len(ret) == 1: return ret[0]
        if len(ret) > 1: return tuple(ret)
    async def getobject(self, name, _info=None):
        ret = type(name, (), {})
        if not _info: _info = await self.call('zmqRemoteApi.info', [name])
        for k, v in _info.items():
            if not isinstance(v, dict): raise ValueError('found nondict')
            if len(v) == 1 and 'func' in v:
                setattr(ret, k, lambda *a, func=f'{name}.{k}': self.call(func, a))
            elif len(v) == 1 and 'const' in v:
                setattr(ret, k, v['const'])
            else:
                setattr(ret, k, self.getobject(f'{name}.{k}', _info=v))
        return ret

async def main():
    client = RemoteAPIClient(verbose=False)
    sim = await client.getobject('sim')
    print(await sim.getObjectHandle('Floor'))
    print(await sim.unpackTable(await sim.packTable({'a': 1, 'b': 2})))
    print(await sim.getObjectHandle('foo'))

if __name__ in ('__main__',):
    asyncio.run(main())
