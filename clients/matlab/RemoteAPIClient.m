% this requires JeroMQ
% call javaaddpath('path/to/jeromq.jar') before running client code

classdef RemoteAPIClient
    properties
        verbose
        ctx
        socket
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

            outputArgs = resp.ret;
        end

        function remoteObject = getObject(obj,name)
            remoteObject = RemoteAPIObject(obj,name);
        end
    end
end

