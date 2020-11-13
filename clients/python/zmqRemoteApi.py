import json
import cbor
import zmq

def deepmapitem(fn, d):
    if isinstance(d, (list, tuple)):
        return type(d)(deepmapitem(fn, x) for x in d)
    elif isinstance(d, dict):
        return {k: deepmapitem(fn, v) for k, v in map(lambda t: fn(*t), d.items())}
    else:
        return d

class RemoteAPIClient:
    def __init__(self, host='localhost', port=23000, *, verbose=False):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.verbose = verbose
        self.socket.connect('tcp://{}:{}'.format(host, port))
    def __del__(self):
        self.socket.close()
        self.context.term()
    def __call__(self, func, args):
        req = {'func': func, 'args': args}
        if self.verbose: print(req)
        self.socket.send(cbor.dumps(req))
        resp = cbor.loads(self.socket.recv())
        resp = deepmapitem(lambda k, v: (k.decode('utf8'), v), resp)
        if self.verbose: print(resp)
        if not resp.get('success', False): raise Exception(resp.get('error'))
        ret = resp['ret']
        if len(ret) == 1: return ret[0]
        if len(ret) > 1: return tuple(ret)

class RemoteAPIFunction:
    def __init__(self, client, funcname):
        self.client, self.funcname = client, funcname
    def __call__(self, *args):
        return self.client(self.funcname, args)
    def __repr__(self):
        return f'RemoteAPIFunction({self.funcname})'

class RemoteAPIObject:
    def __init__(self, client, objname, _info=None):
        self.client, self.objname = client, objname
        if not _info: _info = self.client('zmqRemoteApi.info', [self.objname])
        for k, v in _info.items():
            if not isinstance(v, dict): raise ValueError('found nondict')
            if len(v) == 1 and ('func' in v or 'const' in v):
                setattr(self, k, v.get('const', RemoteAPIFunction(client, f'{objname}.{k}')))
            else:
                setattr(self, k, RemoteAPIObject(client, f'{objname}.{k}', _info=v))
    def __repr__(self):
        return f'RemoteAPIObject({self.objname})'

if __name__ in ('__main__', '__console__'):
    client = RemoteAPIClient()
    sim = RemoteAPIObject(client, 'sim')
if __name__ in ('__main__',):
    print(sim.getObjectHandle('Floor'))
    print(sim.unpackTable(sim.packTable({'a': 1, 'b': 2})))
    print(sim.getObjectHandle('foo'))
