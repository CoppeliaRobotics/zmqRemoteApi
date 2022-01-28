classdef RemoteAPIClient
    properties
        max_recv_sz;
        socket;
    endproperties

    methods
        function client = RemoteAPIClient(host = "localhost", port = 23000, max_recv_sz = 10000000)
            pkg load zeromq
            pkg load json
            client.max_recv_sz = max_recv_sz;
            client.socket = zmq_socket(ZMQ_REQ);
            zmq_connect(client.socket, sprintf("tcp://%s:%d", host, port));
        endfunction

        function outputArgs = call(client, func, inputArgs)
            req = struct("func", func, "args", {inputArgs});
            zmq_send(client.socket, jsonencode(req));
            resp = jsondecode(sprintf("%c", zmq_recv(client.socket, client.max_recv_sz)));
            if resp.success == 0
                error(resp.error);
            endif
            outputArgs = resp.ret;
        endfunction
    endmethods
endclassdef
