classdef RemoteAPIClient < handle
    properties (Access = protected)
        verbose
        ctx
        socket
        max_recv_sz
        uuid
        VERSION
        callbacks
    end

    methods(Static, Access = private)
        function opts = getopts(args, varargin)
            opts = struct();
            for i=1:2:numel(varargin)
                opts.(varargin{i}) = varargin{i+1};
            end
            for i=1:2:numel(args)
                if isfield(opts, args{i})
                    opts.(args{i}) = args{i+1};
                else
                    error('unknown option: "%s"', args{i});
                end
            end
        end
    end

    methods
        function obj = RemoteAPIClient(varargin)
            pkg load zeromq
            pkg load communications

            opts = RemoteAPIClient.getopts(varargin, ...
                'host', 'localhost', ...
                'port', 23000, ...
                'max_recv_sz', 10000000, ...
                'cntPort', -1, ...
                'verbose', false ...
            );

            tcpaddr = @(host, port) sprintf('tcp://%s:%d', host, port);

            obj.verbose = opts.verbose;
            obj.max_recv_sz = opts.max_recv_sz;

            obj.socket = zmq_socket(ZMQ_REQ);
            assert(zmq_connect(obj.socket, tcpaddr(opts.host, opts.port)));

            obj.uuid = sprintf('%04X%04X-%04X-%04X-%04X-%04X%04X%04X', randint(1, 8, 2^16, time()));
            obj.VERSION = 2;
            obj.callbacks = struct();
        end

        function delete(obj)
            req = struct('func', '_*end*_', 'args', {{}}, 'uuid', {obj.uuid}, 'ver', {obj.VERSION}, 'lang', 'octave');
            req_raw = cbor.encode(req);
            zmq_send(obj.socket, req_raw);
            zmq_recv(obj.socket, obj.max_recv_sz);

            zmq_close(obj.socket);
        end

        function addCallback(obj, func, name)
            obj.callbacks.(name) = func;
        end

        function outputArgs = call(obj, fn, inputArgs)
            req = struct('func', fn, 'args', {inputArgs}, 'uuid', {obj.uuid}, 'ver', {obj.VERSION}, 'lang', 'octave', 'argsL', numel(inputArgs));
            req_raw = cbor.encode(req);
            zmq_send(obj.socket, req_raw);

            resp_raw = zmq_recv(obj.socket, obj.max_recv_sz);
            resp = cbor.decode(resp_raw);

            while isfield(resp, 'func')
                args = {};
                % _*repeat*_ not yet implemented!
                if ~strcmp(resp.func, '_*wait*_')
                    if isfield(obj.callbacks, resp.func) % we cannot raise an error if not present: e.g. a custom UI async callback cannot be assigned to a specific client
                        callback = obj.callbacks.(resp.func);
                        try
                            a = callback(resp.args);
                            if ~isempty(a)
                                args = a;
                            end
                        catch ME
                            error('Error in callback: %s', ME.message);
                        end
                    end
                end
                req2 = struct('func', '_*executed*_', 'args', {args}, 'uuid', {obj.uuid}, 'ver', {obj.VERSION}, 'lang', 'octave', 'argsL', numel(args));
                req2_raw = cbor.encode(req2);
                zmq_send(obj.socket, req2_raw);

                resp_raw = zmq_recv(obj.socket, obj.max_recv_sz);
                resp = cbor.decode(resp_raw);
            end
            if isfield(resp, 'err')
                error(resp.err);
            end
            if numel(resp.ret) == 0
                outputArgs = {};
            else
                outputArgs = resp.ret;
            end
        end

        function remoteObject = getObject(obj, name)
            remoteObject = RemoteAPIObject(obj, name);
        end

        function remoteObject = require(obj, name)
            obj.call('zmqRemoteApi.require', {name});
            remoteObject = obj.getObject(name);
        end

        function setStepping(obj, enable)
            if nargin < 2
                enable = true;
            end
            obj.call('sim.setStepping', {enable});
        end

        function step(obj, wait)
            if nargin < 2
                wait = true;
            end
            obj.call('sim.step', {wait});
        end
    end
end
