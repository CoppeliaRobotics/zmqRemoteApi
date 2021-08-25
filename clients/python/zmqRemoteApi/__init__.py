"""CoppeliaSim's Remote API client."""

from time import sleep

import cbor

import zmq


def b64(b):
    import base64
    return base64.b64encode(b).decode('ascii')


class RemoteAPIClient:
    """Client to connect to CoppeliaSim's ZMQ Remote API."""

    def __init__(self, host='localhost', port=23000, *, verbose=False):
        """Create client and connect to the ZMQ Remote API server."""
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.verbose = verbose
        self.socket.connect(f'tcp://{host}:{port}')
        self.sim = None

    def __del__(self):
        """Disconnect and destroy client."""
        self.socket.close()
        self.context.term()

    def call(self, func, args, *, verbose=None):
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
        rawReq = cbor.dumps(req)
        if verbose:
            print(f'Sending raw len={len(rawReq)}, base64={b64(rawReq)}')
        self.socket.send(rawReq)
        rawResp = self.socket.recv()
        if verbose:
            print(f'Received raw len={len(rawResp)}, base64={b64(rawResp)}')
        resp = cbor.loads(rawResp)
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

    def getobject(self, name, _info=None):
        """Retrieve remote object from server."""
        ret = type(name, (), {})
        if not _info:
            _info = self.call('zmqRemoteApi.info', [name])
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

    def call_addon(self, func, *args):
        if self.sim is None:
            self.sim = self.getobject('sim')
        return self.sim.callScriptFunction(f'{func}@ZMQ remote API', self.sim.scripttype_addonscript, *args)

    def setsynchronous(self, enable=True):
        return self.call_addon('setSynchronous', enable)

    def step(self):
        return self.call_addon('step')


if __name__ in ('__main__', '__console__'):
    client = RemoteAPIClient()
    sim = client.getobject('sim')
if __name__ in ('__main__',):
    print(sim.getObjectHandle('Floor'))
    print(sim.unpackTable(sim.packTable({'a': 1, 'b': 2})))
    handles = [sim.createDummy(0.01, 12 * [0]) for _ in range(50)]
    for i, h in enumerate(handles):
        sim.setObjectPosition(h, -1, [0.01 * i, 0.01 * i, 0.01 * i])
    sleep(10)
    for h in handles:
        sim.removeObject(h)


__all__ = ['RemoteAPIClient']
