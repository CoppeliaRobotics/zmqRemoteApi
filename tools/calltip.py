import os
from dataclasses import dataclass, field
from typing import Any

import lark

class CallTipParser:
    class Transformer(lark.Transformer):
        def func_def(self, s):
            return FuncDef(**dict([x for x in s if x]))
        def in_arg_list(self, s):
            return ('in_args', ArgList([x for x in s if x is not None]))
        def func_name(self, s):
            return ('func_name', '.'.join(x.value for x in s))
        def out_arg_list(self, s):
            return ('out_args', ArgList([x for x in s if x is not None]))
        def arg(self, s):
            d = dict([x for y in s for x in y])
            return TableArg(**d) if 'item_type' in d else Arg(**d)
        def arg_with_default(self, s):
            d = dict([x for y in s for x in y])
            return TableArgDef(**d) if 'item_type' in d else ArgDef(**d)
        def varargs(self, _):
            return VarArgs()
        def number(self, n):
            s = n[0]
            if s.endswith('f'):
                s = s[:-1]
            return float(s)
        def string(self, s):
            return s[0][1:-1]
        def true(self, _):
            return True
        def false(self, _):
            return False
        def nil(self, _):
            return None
        def id(self, t):
            return '.'.join(x.value for x in t)
        def map(self, s):
            fs = [x for x in s if x is not None]
            return dict((str(a), b) for a, b in zip(fs[::2], fs[1::2]))
        def array(self, s):
            return list(s)
        def type(self, t):
            return [('type', t[0].value)]
        def array_type(self, t):
            return [('type', 'array'), ('item_type', t[0].value)]
        def array_type_fixed(self, t):
            return [('type', 'array'), ('item_type', t[0].value), ('min_size', t[1].value), ('max_size', t[1].value)]
        def array_type_range(self, t):
            return [('type', 'array'), ('item_type', t[0].value), ('min_size', t[1].value), ('max_size', t[2].value)]
        def array_type_min(self, t):
            return [('type', 'array'), ('item_type', t[0].value), ('min_size', t[1].value)]
        def name(self, t):
            return [('name', t[0].value)]
        def default(self, t):
            return [('default', t[0])]

    def __init__(self):
        with open(os.path.join(os.path.dirname(__file__), 'calltip.lark'), 'rt') as f:
            self.parser = lark.Lark(f, start='func_def')
        self.transformer = CallTipParser.Transformer()

    def parse(self, calltip: str):
        return self.parser.parse(calltip)

    def transform(self, tree):
        d = self.transformer.transform(tree)
        d.validate()
        return d

@dataclass
class Arg:
    name: str
    type: str

    def is_optional(self) -> bool:
        return False

@dataclass
class TableArg(Arg):
    item_type: str = None
    min_size: str = None
    max_size: str = None

@dataclass
class ArgDef(Arg):
    default: Any = None

    def is_optional(self) -> bool:
        return True

@dataclass
class TableArgDef(TableArg):
    default: Any = None

    def is_optional(self) -> bool:
        return True

@dataclass
class VarArgs:
    pass

class ArgList(list):
    def validate(self):
        n = len(self)
        for i, arg in enumerate(self):
            if isinstance(arg, VarArgs) and (i + 1) < n:
                raise Exception('VarArgs must be at last position')

    def is_variadic(self) -> bool:
        if self:
            return isinstance(self[-1], VarArgs)
        else:
            return False

@dataclass
class FuncDef:
    func_name: str
    in_args: ArgList = field(default_factory=ArgList)
    out_args: ArgList = field(default_factory=ArgList)

    def validate(self):
        self.in_args.validate()
        self.out_args.validate()

    _parser = CallTipParser()

    @staticmethod
    def from_calltip(calltip):
        tree = FuncDef._parser.parse(calltip)
        return FuncDef._parser.transform(tree)

    @staticmethod
    def from_calltips_json(path, include_objects=[], exclude_objects=[], exclude_methods=[]):
        import json
        with open(path, 'rt') as f:
            calltips = json.load(f)
        def parse_list(x, sep=','):
            if isinstance(x, str):
                x = [y for y in [z.strip() for z in x.split(sep)] if y != '']
            if isinstance(x, (tuple, list)):
                x = set(x)
            if not isinstance(x, set):
                raise TypeError
            return x
        include_objects, exclude_objects, exclude_methods = map(parse_list, (include_objects, exclude_objects, exclude_methods))
        allowed_object_prefixes = {'sim'}
        ret = {}
        for k, v in calltips.items():
            s = k.split('.')
            if len(s) == 2:
                obj, func = s
                if not any(obj.startswith(p) for p in allowed_object_prefixes):
                    continue
                if include_objects and obj not in include_objects:
                    continue
                elif obj in exclude_objects:
                    continue
                if f'{obj}.{func}' in exclude_methods:
                    continue
                if func[0] == '_':
                    continue
                if obj not in ret:
                    ret[obj] = {}
                try:
                    ret[obj][func] = FuncDef.from_calltip(v.splitlines()[0])
                except lark.exceptions.UnexpectedInput:
                    print(f'error in {obj}')
                    raise
        return ret
