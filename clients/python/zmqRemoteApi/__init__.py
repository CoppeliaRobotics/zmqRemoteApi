"""CoppeliaSim's Remote API client."""

import os

import uuid

from time import sleep

import cbor

import zmq


def b64(b):
    import base64
    return base64.b64encode(b).decode('ascii')


class RemoteAPIClient:
    """Client to connect to CoppeliaSim's ZMQ Remote API."""

    def __init__(self, host='localhost', port=23000, cntport=None, *, verbose=None):
        """Create client and connect to the ZMQ Remote API server."""
        self.verbose = int(os.environ.get('VERBOSE', '0')) if verbose is None else verbose
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.cntsocket = self.context.socket(zmq.SUB)
        self.socket.connect(f'tcp://{host}:{port}')
        self.cntsocket.setsockopt(zmq.SUBSCRIBE, b'')
        self.cntsocket.setsockopt(zmq.CONFLATE, 1)
        self.cntsocket.connect(f'tcp://{host}:{cntport if cntport else port+1}')
        self.uuid=str(uuid.uuid4())

    def __del__(self):
        """Disconnect and destroy client."""
        self.socket.close()
        self.cntsocket.close()
        self.context.term()

    def _send(self, req):
        if self.verbose > 0:
            print('Sending:', req)
        rawReq = cbor.dumps(req)
        if self.verbose > 1:
            print(f'Sending raw len={len(rawReq)}, base64={b64(rawReq)}')
        self.socket.send(rawReq)

    def _recv(self):
        rawResp = self.socket.recv()
        if self.verbose > 1:
            print(f'Received raw len={len(rawResp)}, base64={b64(rawResp)}')
        resp = cbor.loads(rawResp)
        if self.verbose > 0:
            print('Received:', resp)
        return resp

    def _process_response(self, resp):
        if not resp.get('success', False):
            raise Exception(resp.get('error'))
        ret = resp['ret']
        if len(ret) == 1:
            return ret[0]
        if len(ret) > 1:
            return tuple(ret)

    def call(self, func, args):
        """Call function with specified arguments."""
        self._send({'func': func, 'args': args})
        return self._process_response(self._recv())

    def getObject(self, name, _info=None):
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
                setattr(ret, k, self.getObject(f'{name}.{k}', _info=v))
        return ret

    def setStepping(self, enable=True):
        return self.call('setStepping', [enable,self.uuid])

    def step(self, *, wait=True):
        self.getStepCount(False)
        self.call('step', [self.uuid])
        self.getStepCount(wait)

    def getStepCount(self, wait):
        try:
            self.cntsocket.recv(0 if wait else zmq.NOBLOCK)
        except zmq.ZMQError:
            pass


if __name__ == '__console__':
    client = RemoteAPIClient()
    sim = client.getObject('sim')


__all__ = ['RemoteAPIClient']
