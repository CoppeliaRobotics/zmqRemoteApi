classdef RemoteAPIObject
    properties (Access = protected)
        x__client
        x__objectName
    end

    methods
        function self = RemoteAPIObject(client, objectName)
            self.x__client = client;
            self.x__objectName = objectName;
        end

        function varargout = subsref(self, S)
            fn = self.x__objectName;
            args = {};
            func_call = 0;
            for i=1:numel(S)
                if strcmp(S(i).type, '.')
                    fn = strcat(fn, S(i).type, S(i).subs);
                elseif strcmp(S(i).type, '()') && i == numel(S)
                    args = S(i).subs;
                    func_call = 1;
                else
                    error('unexpected sub type %s at %d', S(i).type, i);
                end
            end
            if ~func_call
                args = {fn};
                fn = 'zmqRemoteApi.info';
            end
            varargout = self.x__client.call(fn, args);
        end
    end
end

