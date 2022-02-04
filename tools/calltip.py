import os
from dataclasses import dataclass, field
from typing import Any

import lark

class CallTipParser:
    class Transformer(lark.Transformer):
        def func_def(self, s):
            return FuncDef(**dict([x for x in s if x]))
        def in_arg_list(self, s):
            return ('in_args', [x for x in s if x is not None])
        def func_name(self, s):
            return ('func_name', '.'.join(x.value for x in s))
        def out_arg_list(self, s):
            return ('out_args', [x for x in s if x is not None])
        def arg(self, s):
            d = dict([x for y in s for x in y])
            return TableArg(**d) if 'item_type' in d else Arg(**d)
        def arg_with_default(self, s):
            d = dict([x for y in s for x in y])
            return TableArgDef(**d) if 'item_type' in d else ArgDef(**d)
        def varargs(self, _):
            return VarArgs()
        def number(self, n):
            return float(n[0])
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
            return eval(s[0])
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
        return self.transformer.transform(tree)

@dataclass
class Arg:
    name: str
    type: str

@dataclass
class TableArg(Arg):
    item_type: str = None
    min_size: str = None
    max_size: str = None

@dataclass
class ArgDef(Arg):
    default: Any = None

@dataclass
class TableArgDef(TableArg):
    default: Any = None

@dataclass
class VarArgs:
    pass

@dataclass
class FuncDef:
    func_name: str
    in_args: list = field(default_factory=list)
    out_args: list = field(default_factory=list)
