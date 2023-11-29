% this requires JeroMQ
% call javaaddpath('path/to/jeromq.jar') before running client code

classdef RemoteAPIClient
    properties (Access = protected)
        verbose
        ctx
        socket
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
            try
                ZMsg();
            catch ex
                if ~strcmp(ex.identifier, 'MATLAB:UndefinedFunction')
                    rethrow(ex);
                end
                jeromqVer = '0.5.2';
                jeromqJAR = sprintf('jeromq-%s.jar', jeromqVer);
                if ~isfile(jeromqJAR)
                    mvnRepo = 'https://repo1.maven.org/maven2';
                    websave(jeromqJAR, sprintf('%s/org/zeromq/jeromq/%s/%s', mvnRepo, jeromqVer, jeromqJAR));
                end
                javaaddpath(jeromqJAR);
            end

            import org.zeromq.*;

            opts = RemoteAPIClient.getopts(varargin, ...
                'host', 'localhost', ...
                'port', 23000, ...
                'cntPort', -1, ...
                'verbose', false ...
            );
            tcpaddr = @(host, port) sprintf('tcp://%s:%d', host, port);

            obj.verbose = opts.verbose;

            obj.ctx = ZContext();

            obj.socket = obj.ctx.createSocket(SocketType.REQ);
            obj.socket.connect(java.lang.String(tcpaddr(opts.host, opts.port)));

            obj.uuid = char(java.util.UUID.randomUUID);
            obj.VERSION = 2;
            obj.callbacks = containers.Map();
        end

        function delete(obj)
            req = struct('func', '_*end*_', 'args', {}, 'uuid', {obj.uuid}, 'ver', {obj.VERSION}, 'lang', 'matlab');
            req_raw = cbor.encode(req);
            req_frame = ZFrame(req_raw);
            req_msg = ZMsg();
            req_msg.add(req_frame);
            req_msg.send(obj.socket, 0);
            ZMsg.recvMsg(obj.socket, 0);

            obj.socket.close();
            obj.ctx.close();
        end

        function addCallback(obj, func, name)
            obj.callbacks(name) = func;
        end

        function outputArgs = call(obj, fn, inputArgs)
            % Call function with specified arguments. Is Reentrant
            import org.zeromq.*;
            req = struct('func', fn, 'args', {inputArgs}, 'uuid', {obj.uuid}, 'ver', {obj.VERSION}, 'lang', 'matlab', 'argsL', numel(inputArgs));
            req_raw = cbor.encode(req);
            req_frame = ZFrame(req_raw);
            req_msg = ZMsg();
            req_msg.add(req_frame);
            req_msg.send(obj.socket, 0);

            resp_msg = ZMsg.recvMsg(obj.socket, 0);
            resp_frame = resp_msg.pop();
            resp_raw = typecast(resp_frame.getData(), 'uint8');
            resp = cbor.decode(resp_raw);

            while isfield(resp, 'func')
                args = {};
                % _*repeat*_ not yet implemented!
                if ~strcmp(resp.func, '_*wait*_')
                    if isKey(obj.callbacks, resp.func) % we cannot raise an error if not present: e.g. a custom UI async callback cannot be assigned to a specific client
                        callback = obj.callbacks(resp.func);
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
                req2 = struct('func', '_*executed*_', 'args', {args}, 'uuid', {obj.uuid}, 'ver', {obj.VERSION}, 'lang', 'matlab', 'argsL', numel(args));
                req2_raw = cbor.encode(req2);
                req2_frame = ZFrame(req2_raw);
                req2_msg = ZMsg();
                req2_msg.add(req2_frame);
                req2_msg.send(obj.socket, 0);

                resp_msg = ZMsg.recvMsg(obj.socket, 0);
                resp_frame = resp_msg.pop();
                resp_raw = typecast(resp_frame.getData(), 'uint8');
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
            arguments
                obj (1,1) RemoteAPIClient
                enable (1,1) logical = true
            end

            obj.call('sim.setStepping', {enable});
        end

        function step(obj, wait)
            arguments
                obj (1,1) RemoteAPIClient
                wait (1,1) logical = true
            end

            obj.call('sim.step', {wait});
        end
    end
end

