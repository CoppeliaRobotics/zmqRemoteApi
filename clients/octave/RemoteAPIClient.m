classdef RemoteAPIClient
    properties (Access = protected)
        verbose
        ctx
        socket
        max_recv_sz
        uuid
        cntSocket
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
            if opts.cntPort == -1
                opts.cntPort = opts.port + 1;
            end

            tcpaddr = @(host, port) sprintf('tcp://%s:%d', host, port);

            obj.verbose = opts.verbose;
            obj.max_recv_sz = opts.max_recv_sz;

            obj.socket = zmq_socket(ZMQ_REQ);
            assert(zmq_connect(obj.socket, tcpaddr(opts.host, opts.port)));

            obj.uuid = sprintf('%04X%04X-%04X-%04X-%04X-%04X%04X%04X', randint(1, 8, 2^16, time()));

            obj.cntSocket = zmq_socket(ZMQ_SUB);
            assert(zmq_setsockopt(obj.cntSocket, ZMQ_SUBSCRIBE, ''));
            %assert(zmq_setsockopt(obj.cntSocket, ZMQ_CONFLATE, 1));
            assert(zmq_connect(obj.cntSocket, tcpaddr(opts.host, opts.cntPort)));
        end

        function delete(obj)
            zmq_close(obj.socket)
            zmq_close(obj.cntSocket)
        end

        function outputArgs = call(obj, fn, inputArgs)
            req = struct('func', fn, 'args', {inputArgs});
            req_raw = cbor.encode(req);
            zmq_send(obj.socket, req_raw);

            resp_raw = zmq_recv(obj.socket, obj.max_recv_sz);
            resp = cbor.decode(resp_raw);

            if resp.success == 0
                error(resp.error);
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

        function setStepping(obj, enable)
            if nargin < 2
                enable = true;
            end
            obj.call('setStepping', {enable, obj.uuid});
        end

        function step(obj, wait)
            if nargin < 2
                wait = true;
            end
            obj.getStepCount(false);
            obj.call('step', {obj.uuid});
            obj.getStepCount(wait);
        end

        function getStepCount(obj, wait)
            if wait
                flags = 0;
            else
                flags = ZMQ_DONTWAIT;
            end
            zmq_recv(obj.cntSocket, obj.max_recv_sz, flags);
        end
    end
end
