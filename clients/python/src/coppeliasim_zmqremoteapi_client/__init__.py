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
        def handle_func_arg(arg):
            retArg = arg
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
                    retArg = funcStr + "@func"
            return retArg 

        # convert a possible function to string (up to a depth of 2):
        if 'args' in req and req['args'] != None and isinstance(req['args'], (tuple, list)):
            req['args'] = list(req['args'])
            for i in range(len(req['args'])):
                req['args'][i] = handle_func_arg(req['args'][i]) # depth 1
                if isinstance(req['args'][i], tuple):
                    req['args'][i] = list(req['args'][i])
                if isinstance(req['args'][i], list) and len(req['args'][i]) <= 16: # parse no more than 16 items
                    for j in range(len(req['args'][i])):
                        req['args'][i][j] = handle_func_arg(req['args'][i][j]) #depth 2
                if isinstance(req['args'][i], dict) and len(req['args'][i]) <= 16:
                    req['args'][i] = {key: handle_func_arg(value) for key, value in req['args'][i].items()} #depth 2
            req['argsL'] = len(req['args'])
            
        self.sendCnt = self.sendCnt + 1
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
            ret.copyTable = self.copyTable
            ret.packUInt8Table = self.packUInt8Table
            ret.unpackUInt8Table = self.unpackUInt8Table
            ret.packUInt16Table = self.packUInt16Table
            ret.unpackUInt16Table = self.unpackUInt16Table
            ret.packUInt32Table = self.packUInt32Table
            ret.unpackUInt32Table = self.unpackUInt32Table
            ret.packInt32Table = self.packInt32Table
            ret.unpackInt32Table = self.unpackInt32Table
            ret.packFloatTable = self.packFloatTable
            ret.unpackFloatTable = self.unpackFloatTable
            ret.packDoubleTable = self.packDoubleTable
            ret.unpackDoubleTable = self.unpackDoubleTable

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

    def copyTable(self, table):
        import copy 
        return copy.deepcopy(table)
        
    def _packXTable(self, table, w, start, cnt):
        import array
        if cnt == 0:
            cnt = len(table) - start
        arr = array.array(w, table[start:(start + cnt)])
        return arr.tobytes()

    def _unpackXTable(self, data, w, start, cnt, off):
        import array
        arr = array.array(w)
        start *= arr.itemsize
        start += off
        if cnt == 0:
            cnt =  len(data) - start
        else:
            cnt *= arr.itemsize
        arr.frombytes(data[start:(start + cnt)])
        return list(arr)

    def packUInt8Table(self, table, start=0, cnt=0):
        return self._packXTable(table, 'B', start, cnt)

    def unpackUInt8Table(self, data, start=0, cnt=0, off=0):
        return self._unpackXTable(data, 'B', start, cnt, off)
        
    def packUInt16Table(self, table, start=0, cnt=0):
        return self._packXTable(table, 'H', start, cnt)
        
    def unpackUInt16Table(self, data, start=0, cnt=0, off=0):
        return self._unpackXTable(data, 'H', start, cnt, off)
        
    def packUInt32Table(self, table, start=0, cnt=0):
        return self._packXTable(table, 'L', start, cnt)
        
    def unpackUInt32Table(self, data, start=0, cnt=0, off=0):
        return self._unpackXTable(data, 'L', start, cnt, off)
        
    def packInt32Table(self, table, start=0, cnt=0):
        return self._packXTable(table, 'l', start, cnt)
        
    def unpackInt32Table(self, data, start=0, cnt=0, off=0):
        return self._unpackXTable(data, 'l', start, cnt, off)
        
    def packFloatTable(self, table, start=0, cnt=0):
        return self._packXTable(table, 'f', start, cnt)
        
    def unpackFloatTable(self, data, start=0, cnt=0, off=0):
        return self._unpackXTable(data, 'f', start, cnt, off)
        
    def packDoubleTable(self, table, start=0, cnt=0):
        return self._packXTable(table, 'd', start, cnt)

    def unpackDoubleTable(self, data, start=0, cnt=0, off=0):
        return self._unpackXTable(data, 'd', start, cnt, off)
        
    def setStepping(self, enable=True):  # for backw. comp., now via sim.setStepping
        return self.call('sim.setStepping', [enable])

    def step(self, *, wait=True):  # for backw. comp., now via sim.step
        self.call('sim.step', [wait])

if __name__ == '__console__':
    client = RemoteAPIClient()

__all__ = ['RemoteAPIClient']
