% this requires JeroMQ
% call javaaddpath('path/to/jeromq.jar') before running client code

classdef RemoteAPIClient
    properties (Access = protected)
        verbose
        ctx
        socket
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
            if opts.cntPort == -1
                opts.cntPort = opts.port + 1;
            end

            tcpaddr = @(host, port) sprintf('tcp://%s:%d', host, port);

            obj.verbose = opts.verbose;

            obj.ctx = ZContext();

            obj.socket = obj.ctx.createSocket(SocketType.REQ);
            obj.socket.connect(java.lang.String(tcpaddr(opts.host, opts.port)));

            obj.uuid = char(java.util.UUID.randomUUID);

            obj.cntSocket = obj.ctx.createSocket(SocketType.SUB);
            obj.cntSocket.subscribe(java.lang.String(''));
            obj.cntSocket.setConflate(true);
            obj.cntSocket.connect(java.lang.String(tcpaddr(opts.host, opts.cntPort)));
        end

        function delete(obj)
            obj.socket.close();
            obj.cntSocket.close();
            obj.ctx.close();
        end

        function outputArgs = call(obj, fn, inputArgs)
            import org.zeromq.*;

            req = struct('func', fn, 'args', {inputArgs});
            req_raw = cbor.encode(req);
            req_frame = ZFrame(req_raw);
            req_msg = ZMsg();
            req_msg.add(req_frame);
            req_msg.send(obj.socket, 0);

            resp_msg = ZMsg.recvMsg(obj.socket, 0);
            resp_frame = resp_msg.pop();
            resp_raw = typecast(resp_frame.getData(), 'uint8');
            resp = cbor.decode(resp_raw);

            if resp.success == 0
                error(resp.error)
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
            arguments
                obj (1,1) RemoteAPIClient
                enable (1,1) logical = true
            end

            obj.call('setStepping', {enable, obj.uuid});
        end

        function step(obj, wait)
            arguments
                obj (1,1) RemoteAPIClient
                wait (1,1) logical = true
            end

            obj.getStepCount(false);
            obj.call('step', {obj.uuid});
            obj.getStepCount(wait);
        end

        function getStepCount(obj, wait)
            import org.zeromq.*;

            if wait
                flags = 0;
            else
                flags = ZMQ.DONTWAIT;
            end
            obj.cntSocket.recv(flags);
        end
    end
end

