"""CoppeliaSim's Remote API client."""

import cbor

import zmq


class RemoteAPIClient:
    """Client to connect to CoppeliaSim's ZMQ Remote API."""

    def __init__(self, host='localhost', port=23000, *, verbose=False):
        """Create client and connect to the ZMQ Remote API server."""
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.verbose = verbose
        self.socket.connect(f'tcp://{host}:{port}')

    def __del__(self):
        """Disconnect and destroy client."""
        self.socket.close()
        self.context.term()

    def call(self, func, args, verbose=None):
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
        self.socket.send(cbor.dumps(req))
        resp = cbor.loads(self.socket.recv())
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


if __name__ in ('__main__', '__console__'):
    client = RemoteAPIClient()
    sim = client.getobject('sim')
if __name__ in ('__main__',):
    print(sim.getObjectHandle('Floor'))
    print(sim.unpackTable(sim.packTable({'a': 1, 'b': 2})))
    print(sim.getObjectHandle('foo'))
