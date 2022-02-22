% this requires JeroMQ
% call javaaddpath('path/to/jeromq.jar') before running client code

classdef RemoteAPIClient
    properties
        verbose
        ctx
        socket
        uuid
        cntSocket
    end

    methods
        function obj = RemoteAPIClient(host,port,verbose)
            import org.zeromq.*;
            if nargin<1; host = 'localhost'; end
            if nargin<2; port = 23000; end
            if nargin<3; verbose = 0; end
            obj.verbose = verbose;
            obj.ctx = ZContext();
            obj.socket = obj.ctx.createSocket(SocketType.REQ);
            addr = sprintf('tcp://%s:%d',host,port);
            addr = java.lang.String(addr);
            obj.socket.connect(addr);

            obj.uuid = java.util.UUID.randomUUID;
            obj.cntSocket = obj.ctx.createSocket(SocketType.SUB);
            cntAddr = sprintf('tcp://%s:%d',host,port+1);
            cntAddr = java.lang.String(cntAddr);
            obj.cntSocket.subscribe(java.lang.String(''));
            obj.cntSocket.setConflate(true);
            obj.cntSocket.connect(cntAddr);
        end

        function delete(obj)
            obj.socket.close();
            obj.ctx.close();
        end

        function outputArgs = call(obj,fn,inputArgs)
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

            if resp.success==0
                error(resp.error)
            end

            if numel(resp.ret) == 0
                outputArgs = {};
            else
                outputArgs = resp.ret;
            end
        end

        function remoteObject = getObject(obj,name)
            remoteObject = RemoteAPIObject(obj,name);
        end

        function setStepping(obj,enable)
            arguments
                obj (1,1) RemoteAPIClient
                enable (1,1) logical = true
            end
            obj.call('setStepping', {enable, char(obj.uuid)});
        end

        function step(obj,wait)
            arguments
                obj (1,1) RemoteAPIClient
                wait (1,1) logical = true
            end
            obj.getStepCount(false);
            obj.call('step', {char(obj.uuid)});
            obj.getStepCount(wait);
        end

        function getStepCount(obj,wait)
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

