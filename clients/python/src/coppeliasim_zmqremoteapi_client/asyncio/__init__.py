"""CoppeliaSim's Remote API client."""

import asyncio
import sys
import os
import re
import uuid

from contextlib import contextmanager

try:
    import cbor2 as cbor
except ModuleNotFoundError:
    import cbor

import zmq
import zmq.asyncio


if sys.platform == 'win32' and sys.version_info >= (3, 8, 0):
    if isinstance(asyncio.get_event_loop_policy(), asyncio.windows_events.WindowsProactorEventLoopPolicy):
        print("""

    WARNING: on Windows and Python 3.8+, `asyncio` might not work properly; in case, add the following before `asyncio.run(...)`:

    if sys.platform == 'win32' and sys.version_info >= (3, 8, 0):
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

""")


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
        self.host, self.port, self.cntport = host, port, cntport or port + 1
        self.uuid = str(uuid.uuid4())
        self.callbackFuncs = {}
        self.requiredItems = {}
        self.VERSION = 2
        main_globals = sys.modules['__main__'].__dict__
        main_globals['require'] = self.require
        # multiple sockets will be created for multiple concurrent requests, as needed
        self.sockets = []

    async def __aenter__(self):
        """Add one socket to the pool."""
        self.context = zmq.asyncio.Context()
        return self

    async def __aexit__(self, *excinfo):
        """Disconnect and destroy client."""
        for socket in self.sockets:
            socket.close()
        self.context.term()

    @contextmanager
    def _socket(self):
        if not self.sockets:
            socket = self.context.socket(zmq.REQ)
            socket.connect(f'tcp://{self.host}:{self.port}')
            if self.verbose > 0:
                print('Added a new socket:', socket)
        else:
            socket = self.sockets.pop()
            if self.verbose > 0:
                print('Reusing existing socket:', socket)
        try:
            yield socket
        finally:
            self.sockets.append(socket)

    async def _send(self, socket, req):
        # convert a possible function to string:
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
        req['ver'] = self.VERSION
        req['lang'] = 'python'
        if self.verbose > 0:
            print('Sending:', req, socket)
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
        await socket.send(rawReq)

    async def _recv(self, socket):
        rawResp = await socket.recv()
        if self.verbose > 1:
            print(f'Received raw len={len(rawResp)}, base64={b64(rawResp)}')
        resp = cbor.loads(rawResp)
        if self.verbose > 0:
            print('Received:', resp, socket)
        return resp

    def _process_response(self, resp):
        ret = resp['ret']
        if len(ret) == 1:
            return ret[0]
        if len(ret) > 1:
            return tuple(ret)

    async def call(self, func, args):
        # Call function with specified arguments. Is reentrant
        with self._socket() as socket:
            await self._send(socket, {'func': func, 'args': args})
            reply = await self._recv(socket)
            
            while isinstance(reply, dict) and 'func' in reply:
                # We have a callback or a wait/repeat:
                if reply['func'] == '_*wait*_':
                    func = '_*executed*_'
                    args = []
                    await self._send(socket, {'func': func, 'args': args})
                elif reply['func'] == '_*repeat*_':
                    await self._send(socket, {'func': func, 'args': args})
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
                    await self._send(socket, {'func': func, 'args': args})
                reply = await self._recv(socket)
                
            if 'err' in reply:
                raise Exception(reply.get('err'))  # __EXCEPTION__
            return self._process_response(reply)

    async def getObject(self, name, _info=None):
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
                setattr(ret, k, self.getObject(f'{name}.{k}', _info=v))
        if name == 'sim':
            ret.getScriptFunctions = self.getScriptFunctions
        return ret

    async def require(self, name):
        if name in self.requiredItems:
            ret = self.requiredItems[name]
        else:
            await self.call('zmqRemoteApi.require', [name])
            ret = await self.getObject(name)
        return ret

    async def getScriptFunctions(self, scriptHandle):

        async def inner_function(func, *args):
            return await self.call('sim.callScriptFunction', (func, scriptHandle) + args)

        def outer_function(func):
            async def wrapper(*args):
                return await inner_function(func, *args)
            return wrapper

        def custom_getattr(d, func):
            return outer_function(func)

        return type('ScriptFunctionWrapper', (object,), {'__getattr__': custom_getattr})()
        
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
        
    async def setStepping(self, enable=True):  # for backw. comp., now via sim.setStepping
        return await self.call('sim.setStepping', [enable])

    async def step(self, *, wait=True):  # for backw. comp., now via sim.step
        await self.call('sim.step', [wait])


__all__ = ['RemoteAPIClient']
