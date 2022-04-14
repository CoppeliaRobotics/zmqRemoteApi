def cpp_type(arg):
    def _(t):
        if t == 'array':
            return f'std::vector<{_(arg.item_type)}>'
        if t == 'buffer':
            return 'std::vector<uint8_t>'
        if t == 'int':
            return 'int64_t'
        if t == 'float':
            return 'double'
        if t == 'string':
            return 'std::string'
        if t == 'func':
            return 'std::string'
        if t == 'map':
            return 'json'
        if t == 'any':
            return 'json'
        return t
    if arg.is_optional():
        return f'std::optional<{_(arg.type)}>'
    else:
        return _(arg.type)

def cpp_types(args):
    return ', '.join(map(cpp_type, args))

def cpp_arg(arg):
    return f'{cpp_type(arg)} {arg.name}'

def cpp_args(args):
    return ', '.join(map(cpp_arg, args))

def cpp_arg_d(arg):
    if arg.is_optional():
        return f'{cpp_type(arg)} {arg.name} = {{}}'
    else:
        return cpp_arg(arg)

def cpp_args_d(args):
    return ', '.join(map(cpp_arg_d, args))

def cpp_rets(args):
    if len(args) == 0:
        return 'void'
    if len(args) == 1:
        return cpp_type(args[0])
    return f'std::tuple<{cpp_types(args)}>'
