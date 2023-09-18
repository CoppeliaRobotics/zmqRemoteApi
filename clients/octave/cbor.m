% cbor.m - CBOR (RFC 7049) encoder/decoder for MATLAB
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
% cbor.m is courtesy of Federico Ferri

classdef cbor
    methods
        function obj = cbor()
            fprintf('Running tests...\n');
            cbor.test
            clear obj
        end
    end

    properties(Constant, Access = private)
        MASK_MAJOR = 0xE0
        MASK_INFO = 0x1F
        UNSIGNED_INT = 0
        NEGATIVE_INT = 1
        BYTE_STRING = 2
        TEXT_STRING = 3
        ARRAY = 4
        MAP = 5
        SIMPLE_VALUE = 7
        FALSE = 20
        TRUE = 21
        NULL = 22
        UNDEFINED = 23
        SINGLE_BYTE_EXTENSION = 24
        FLOAT16 = 25
        FLOAT32 = 26
        FLOAT64 = 27
        BREAK = 31
        UINT8 = 24
        UINT16 = 25
        UINT32 = 26
        UINT64 = 27
        UNDEFINED_LENGTH = 31
    end

    methods(Static, Access = private)
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
    end

    methods(Static)
        function [o,d1] = decode(d,varargin)
            opts = cbor.getopts(varargin, ...
                'array2mat', false ...
            );
            assert(isa(d, 'uint8'));
            major = bitshift(bitand(d(1), cbor.MASK_MAJOR), -5);
            info = bitand(d(1), cbor.MASK_INFO);
            d1 = d(2:end);
            d1 = d1(:)';
            as = @(x,t) typecast(flip(x), t);
            if major ~= cbor.SIMPLE_VALUE
                if info < 24
                    info1 = double(info);
                elseif info == cbor.UINT8
                    info1 = double(d1(1));
                    d1 = d1(2:end);
                elseif info == cbor.UINT16
                    info1 = double(as(d1(1:2), 'uint16'));
                    d1 = d1(3:end);
                elseif info == cbor.UINT32
                    info1 = double(as(d1(1:4), 'uint32'));
                    d1 = d1(5:end);
                elseif info == cbor.UINT64
                    info1 = double(as(d1(1:8), 'uint64'));
                    d1 = d1(9:end);
                elseif info == cbor.UNDEFINED_LENGTH
                    info1 = NaN;
                elseif info > 27
                    error('malformed data (info > 27)');
                end
            end
            if major == cbor.UNSIGNED_INT
                if isnan(info1) error('malformed data (int + undefined length)') end
                o = info1;
            elseif major == cbor.NEGATIVE_INT
                if isnan(info1) error('malformed data (negative int + undefined length)') end
                o = -1 - info1;
            elseif major == cbor.BYTE_STRING
                if isnan(info1) error('undefined length byte strings are not supported by this decoder') end
                o = d1(1:info1);
                d1 = d1((info1+1):end);
            elseif major == cbor.TEXT_STRING
                if isnan(info1) error('undefined length text strings are not supported by this decoder') end
                o = native2unicode(d1(1:info1), 'UTF-8');
                d1 = d1((info1+1):end);
            elseif major == cbor.ARRAY
                if isnan(info1) info1 = 1e10; end
                o = {};
                homogeneous = true;
                for i=1:info1
                    if d1(1) == 255 d1 = d1(2:end); break end
                    [o{i}, d1] = cbor.decode(d1);
                    if homogeneous && i > 1 && ~strcmp(class(o{1}),class(o{i}))
                        homogeneous = false;
                    end
                end
                if opts.array2mat && homogeneous && (numel(o) == 0 || ~isa(o(1),'char'))
                    o = cell2mat(o);
                end
            elseif major == cbor.MAP
                if isnan(info1) info1 = 1e10; end
                n = {};
                v = {};
                for i=1:info1
                    if d1(1) == 255 d1 = d1(2:end); break end
                    [n{i}, d1] = cbor.decode(d1);
                    [v{i}, d1] = cbor.decode(d1);
                    if n{i}(1) == '_'
                        % field names cannot start with underscore
                        n{i} = sprintf('x%s', n{i});
                    end
                end
                o = cell2struct(v, n, 2);
            elseif major == cbor.SIMPLE_VALUE
                if info == cbor.SINGLE_BYTE_EXTENSION
                    % simple value 32..255 in next byte
                    info = d1(1);
                    d1 = d1(2:end);
                    if info < 32 error('malformed simple value (0..31 in byte ext)') end
                end

                if info == cbor.FALSE
                    o = false;
                elseif info == cbor.TRUE
                    o = true;
                elseif info == cbor.NULL
                    o = NaN;
                elseif info == cbor.UNDEFINED
                    error('cannot decode undefined value');
                elseif info == cbor.FLOAT16
                    f16 = as(d1(1:2), 'uint16');
                    d1 = d1(3:end);
                    % due to lack of 16-bit floats support, we decode
                    % IEEE 754 16-bit float manually:
                    sign = (-1) ^ double(bitget(f16,16));
                    expo = bitshift(bitand(f16,0x7C00),-10);
                    frac = bitand(f16,0x03FF);
                    if expo == 0
                        o = sign * 2^-14 * double(frac) / 1024;
                    elseif expo == 0x1F && frac == 0
                        o = sign * inf;
                    elseif expo == 0x1F && frac ~= 0
                        o = nan;
                    else
                        o = sign * 2^(double(expo) - 15) * (1 + double(frac) / 1024);
                    end
                elseif info == cbor.FLOAT32
                    o = as(d1(1:4), 'single');
                    d1 = d1(5:end);
                elseif info == cbor.FLOAT64
                    o = as(d1(1:8), 'double');
                    d1 = d1(9:end);
                elseif info == cbor.BREAK
                    % not used - will peek for 255 manually where needed
                    error('unexpected break');
                else
                    error('unrecognized simple value: %d', info);
                end
            end
        end

        function [d] = encode(o,varargin)
            opts = cbor.getopts(varargin ...
            );
            HDR = @(major, info) cbor.encode({'@HDR', {}; major, info});
            tobytes = @(x) flip(typecast(x, 'uint8'));

            % encoding of header with simple value (length, integers, etc...):
            if isa(o, 'cell') && isequal(size(o), [2, 2]) && strcmp(o{1,1}, '@HDR')
                major = o{2,1};
                info = o{2,2};
                if info < 0
                    error('info value cannot be negative');
                elseif info < 24 || major == cbor.SIMPLE_VALUE
                    d = [uint8(info)];
                elseif info < 2^8
                    d = [cbor.UINT8, uint8(info)];
                elseif info < 2^16
                    d = [cbor.UINT16, tobytes(uint16(info))];
                elseif info < 2^32
                    d = [cbor.UINT32, tobytes(uint32(info))];
                elseif info < 2^64
                    d = [cbor.UINT64, tobytes(uint64(info))];
                else
                    error('info value too big: %d', info);
                end
                d(1) = uint8( ...
                    bitor( ...
                        bitshift(uint8(major), 5), ...
                        bitand(cbor.MASK_INFO, uint8(d(1))) ...
                    ) ...
                );
                return
            end

            % encode NaN as null
            if isnumeric(o) && numel(o) == 1 && isnan(o)
                d = [HDR(cbor.SIMPLE_VALUE, cbor.NULL)];
                return
            end

            % array types
            l = numel(o);
            if l == 0
                d = [HDR(cbor.ARRAY, 0)];
                return
            elseif (iscell(o) || l > 1) && isvector(o)
                if isa(o, 'char')
                    % char vector is handled later as a string
                elseif isa(o, 'uint8')
                    % uint8 vector is used for binary data
                    d = [HDR(cbor.BYTE_STRING, l), o(:)'];
                    return
                elseif iscell(o)
                    % cell array encodes as array
                    d = [HDR(cbor.ARRAY, l)];
                    for i=1:l; d = [d, cbor.encode(o{i})]; end
                    return
                else
                    % regular vector encodes as array
                    d = [HDR(cbor.ARRAY, l)];
                    for i=1:l; d = [d, cbor.encode(o(i))]; end
                    return
                end
            elseif l > 1
                error('cannot encode nd-matrix');
            end

            % basic types:
            if isa(o, 'struct')
                % map
                n = fieldnames(o);
                info = struct2cell(o);
                l = numel(n);
                d = [HDR(cbor.MAP, l)];
                for i=1:l; d = [d, cbor.encode(n{i}), cbor.encode(info{i})]; end
            elseif isa(o, 'single')
                if mod(o, 1) == 0; d = cbor.encode(int64(o)); return; end
                d = [HDR(cbor.SIMPLE_VALUE, cbor.FLOAT32), tobytes(o)];
            elseif isa(o, 'double')
                if mod(o, 1) == 0; d = cbor.encode(int64(o)); return; end
                d = [HDR(cbor.SIMPLE_VALUE, cbor.FLOAT64), tobytes(o)];
            elseif isa(o, 'integer')
                if o < 0
                    d = [HDR(cbor.NEGATIVE_INT, -1 - o)];
                else
                    d = [HDR(cbor.UNSIGNED_INT, o)];
                end
            elseif isa(o, 'char')
                u = unicode2native(o, 'UTF-8');
                l = numel(u);
                d = [HDR(cbor.TEXT_STRING, l), u];
            elseif isa(o, 'string')
                d = cbor.encode(char(o));
            elseif isa(o, 'logical')
                if o
                    d = [HDR(cbor.SIMPLE_VALUE, cbor.TRUE)];
                else
                    d = [HDR(cbor.SIMPLE_VALUE, cbor.FALSE)];
                end
            else
                error('unsupported type: %s', class(o));
            end
        end

        function test()
            % encode/decode to hex-string:
            enc = @(o) sprintf('%02x', cbor.encode(o));
            dec = @(s) cbor.decode(uint8(sscanf(s, '%02x')'));

            % like isequaln(), but can mix cell with num arrays
            isequalx = @(a,b) isequaln(a,b) ...
                || (iscell(a) && ~iscell(b) && isequaln(a,num2cell(b))) ...
                || (~iscell(a) && iscell(b) && isequaln(num2cell(a),b));

            T = @(a,b) assert(strcmpi(enc(a),b) && isequalx(a,dec(b)));

            T(false,'F4');
            T(true,'F5');
            T(NaN,'F6');

            T(1,'01');
            T(100,'1864');
            T(1000,'1903E8');
            T(1000000,'1A000F4240');
            T(1000000000000,'1B000000E8D4A51000');
            T(23,'17');
            T(24,'1818');
            T(32,'1820');
            T(2^8,'190100');
            T(2^16,'1A00010000');
            T(2^32,'1B0000000100000000');

            T(3.14159,'FB400921F9F01B866E');

            T('abc','63616263');

            T(struct('a', 10.52),'A16161FB40250A3D70A3D70A');
            T(struct('b', true),'A16162F5');
            T(struct('c', 'foo'),'A1616363666F6F');
            T(struct('a', 10.52, 'b', true, 'c', 'foo'),'A36161FB40250A3D70A3D70A6162F5616363666F6F');
            T(struct('x', struct('y', struct('z', 42))),'A16178A16179A1617A182A');

            T({1,2,3},'83010203');
            T([1,2,3],'83010203');

            T({1.23,4.56,7.89},'83FB3FF3AE147AE147AEFB40123D70A3D70A3DFB401F8F5C28F5C28F');

            T(num2cell(2.^(0:16)),'9101020408101820184018801901001902001904001908001910001920001940001980001A00010000');

            %!OCTAVE!%T(["a","b","c"],'83616161626163');
            T({"a","b","c"},'83616161626163');
            T({'a','b','c'},'83616161626163');

            T({0,"b",true,-1,3.98},'85006162F520FB400FD70A3D70A3D7');

            T(uint8([1,2,3]),'43010203');
            T(uint8(zeros([1,32])),'58200000000000000000000000000000000000000000000000000000000000000000');

            T({true,false,false,true,false},'85F5F4F4F5F4');
            T([true,false,false,true,false],'85F5F4F4F5F4');

            T([NaN,NaN],'82F6F6');

            T({1},'8101');

            % testcases from https://en.wikipedia.org/wiki/Half-precision_floating-point_format#Half_precision_examples
            Td = @(a,b) assert(abs(a-dec(b))<1e-8);
            Td(1.5,'F93E00');
            Td(0,'F90000');
            Td(0.000000059604645,'F90001');
            Td(0.000060975552,'F903FF');
            Td(0.00006103515625,'F90400');
            Td(0.33325195,'F93555');
            Td(0.99951172,'F93BFF');
            Td(1,'F93C00');
            Td(1.00097656,'F93C01');
            Td(65504,'F97BFF');
            assert(inf==dec('F97C00'));
            assert(-inf==dec('F9FC00'));
            Td(0,'F98000'); % -0
            Td(-2,'F9C000');

            % test for more than 24 items:

            T(struct('a', 1000, 'b', 1001, 'c', 1002, 'd', 1003, 'e', 1004, 'f', 1005, 'g', 1006, 'h', 1007, 'i', 1008, 'j', 1009, 'k', 1010, 'l', 1011, 'm', 1012, 'n', 1013, 'o', 1014, 'p', 1015, 'q', 1016, 'r', 1017, 's', 1018, 't', 1019, 'u', 1020, 'v', 1021, 'w', 1022, 'x', 1023, 'y', 1024, 'z', 1025),'B81A61611903E861621903E961631903EA61641903EB61651903EC61661903ED61671903EE61681903EF61691903F0616A1903F1616B1903F2616C1903F3616D1903F4616E1903F5616F1903F661701903F761711903F861721903F961731903FA61741903FB61751903FC61761903FD61771903FE61781903FF6179190400617A190401');

            % indefinite-length array and map items: (decode-only)

            assert(isequalx(struct('a', 1, 'b', 2), dec('BF616101616202FF')));
            assert(isequalx([1,2,3], dec('9F010203FF')));

            fprintf('All tests passed successfully.\n');
        end
    end
end
