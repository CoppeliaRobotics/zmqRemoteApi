classdef RemoteAPIObject
    properties (Access = protected)
        x__client
        x__objectName
        x__info
    end

    methods
        function self = RemoteAPIObject(client, objectName)
            self.x__client = client;
            self.x__objectName = objectName;
            info = client.call('zmqRemoteApi.info', {objectName});
            self.x__info = info{1};
        end

        function varargout = subsref(self, S)
            for i=1:numel(S)-1
                if ~strcmp(S(i).type, '.')
                    error('invalid field access');
                end
            end
            switch S(end).type
                case '.'
                    info = builtin('subsref', self.x__info, S);
                    if ~isfield(info, 'const')
                        error('not a constant');
                    end
                    varargout = {info.const};
                case '()'
                    info = builtin('subsref', self.x__info, S(1:end-1));
                    if ~isfield(info, 'func')
                        error('not a function');
                    end
                    fn = self.x__objectName;
                    for i=1:numel(S)-1
                        fn = strcat(fn, S(i).type, S(i).subs);
                    end
                    args = S(end).subs;
                    varargout = self.x__client.call(fn, args);
                otherwise
                    error('invalid field access');
            end
        end
    end
end

