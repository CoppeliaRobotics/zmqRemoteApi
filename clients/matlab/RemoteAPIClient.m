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
            req_msg = ZMsg.newStringMsg(jsonencode(req));
            req_msg.send(obj.socket, 0);

            resp_msg = ZMsg.recvMsg(obj.socket, 0);
            resp_raw = char(resp_msg.popString());
            resp = jsondecode(resp_raw);

            if resp.success==0
                error(resp.error)
            end

            outputArgs = resp.ret;
        end
    end
end

