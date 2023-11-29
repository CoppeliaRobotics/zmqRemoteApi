"""CoppeliaSim's Remote API client."""

import os
import re
import sys
import uuid

import zmq

try:
    import cbor2 as cbor
except ModuleNotFoundError:
    import cbor


def b64(b):
    import base64
    return base64.b64encode(b).decode('ascii')


def _getFuncIfExists(name):
    method = None
    try:
        main_globals = sys.modules['__main__'].__dict__
        method = main_globals[name]
    except BaseException:
        pass
    return method


def cbor_encode_anything(encoder, value):
    if 'numpy' in sys.modules:
        import numpy as np
        if np.issubdtype(type(value), np.floating):
            value = float(value)
        if isinstance(value, np.ndarray):
            value = value.tolist()
    return encoder.encode(value)


class RemoteAPIClient:
    """Client to connect to CoppeliaSim's ZMQ Remote API."""

    def __init__(self, host='localhost', port=23000, cntport=None, *, verbose=None):
        """Create client and connect to the ZMQ Remote API server."""
        self.verbose = int(os.environ.get('VERBOSE', '0')) if verbose is None else verbose
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(f'tcp://{host}:{port}')
        self.uuid = str(uuid.uuid4())
        self.callbackFuncs = {}
        self.requiredItems = {}
        self.VERSION = 2
        self.timeout = 10 * 60
        self.sendCnt = 0
        main_globals = sys.modules['__main__'].__dict__
        main_globals['require'] = self.require

    def __del__(self):
        """Disconnect and destroy client."""
        # here we can't use self.socket anymore (i.e. sending). Instead we have a timeout on the CoppeliaSim side
        self.socket.close()
        self.context.term()
        
    def _send(self, req):
        # convert a possible function to string:
        self.sendCnt = self.sendCnt + 1
        if 'args' in req and isinstance(req['args'], (tuple, list)):
            req['args'] = list(req['args'])
            for i, arg in enumerate(req['args']):
                if callable(arg):
                    funcStr = str(arg)
                    m = re.search(r"<function (.+) at 0x([0-9a-fA-F]+)(.*)", funcStr)
                    if m:
                        funcStr = m.group(1) + '_' + m.group(2)
                    else:
                        m = re.search(r"<(.*)method (.+) of (.+) at 0x([0-9a-fA-F]+)(.*)", funcStr)
                        if m:
                            funcStr = m.group(2) + '_' + m.group(4)
                        else:
                            funcStr = None
                    if funcStr:
                        self.callbackFuncs[funcStr] = arg
                        req['args'][i] = funcStr + "@func"
            req['argsL'] = len(req['args'])
        req['uuid'] = self.uuid
        if self.sendCnt == 1:
            req['ver'] = self.VERSION
            req['lang'] = 'python'
            req['timeout'] = self.timeout
        if self.verbose > 0:
            print('Sending:', req)
        try:
            kwargs = {}
            if cbor.__package__ == 'cbor2':
                # only 'cbor2' has a 'default' kwarg:
                kwargs['default'] = cbor_encode_anything
            rawReq = cbor.dumps(req, **kwargs)
        except Exception as err:
            raise Exception("illegal argument " + str(err))  # __EXCEPTION__
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
        ret = resp['ret']
        if len(ret) == 1:
            return ret[0]
        if len(ret) > 1:
            return tuple(ret)

    def call(self, func, args):
        # Call function with specified arguments. Is reentrant
        self._send({'func': func, 'args': args})
        reply = self._recv()
        while isinstance(reply, dict) and 'func' in reply:
            # We have a callback or a wait/repeat:
            if reply['func'] == '_*wait*_':
                func = '_*executed*_'
                args = []
                self._send({'func': func, 'args': args})
            else:
                if reply['func'] == '_*repeat*_':
                    self._send({'func': func, 'args': args})
                else:
                    if reply['func'] in self.callbackFuncs:
                        args = self.callbackFuncs[reply['func']](*reply['args'])
                    else:
                        funcToRun = _getFuncIfExists(reply['func'])
                        if funcToRun is not None:  # we cannot raise an error: e.g. a custom UI async callback cannot be assigned to a specific client
                            args = funcToRun(*reply['args'])
                    if args is None:
                        args = []
                    if not isinstance(args, list):
                        args = [args]
                    func = '_*executed*_'
                    self._send({'func': func, 'args': args})
            reply = self._recv()
        if 'err' in reply:
            raise Exception(reply.get('err'))  # __EXCEPTION__
        return self._process_response(reply)

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

        if name == 'sim':
            ret.getScriptFunctions = self.getScriptFunctions

        return ret

    def require(self, name):
        if name in self.requiredItems:
            ret = self.requiredItems[name]
        else:
            self.call('zmqRemoteApi.require', [name])
            ret = self.getObject(name)
        return ret

    def getScriptFunctions(self, scriptHandle):
        return type('', (object,), {
            '__getattr__':
                lambda _, func:
                    lambda *args:
                        self.call('sim.callScriptFunction', (func, scriptHandle) + args)
        })()

    def setStepping(self, enable=True):  # for backw. comp., now via sim.setStepping
        return self.call('sim.setStepping', [enable])

    def step(self, *, wait=True):  # for backw. comp., now via sim.step
        self.call('sim.step', [wait])


if __name__ == '__console__':
    client = RemoteAPIClient()

__all__ = ['RemoteAPIClient']
