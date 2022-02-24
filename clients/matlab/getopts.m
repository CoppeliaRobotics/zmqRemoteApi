% getopts.m - Simple key/value parser for varargin functions
% Copyright (C) 2022 Coppelia Robotics AG
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%
% getopts.m is courtesy of Federico Ferri

function opts = getopts(args,varargin)
    opts = struct();
    for i=1:2:numel(varargin)
        opts.(varargin{i}) = varargin{i+1};
    end
    for i=1:2:numel(args)
        if isfield(opts,args{i})
            opts.(args{i}) = args{i+1};
        else
            error('unknown option: "%s"',args{i});
        end
    end
end
