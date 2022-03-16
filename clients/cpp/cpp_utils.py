def cpp_type(arg):
    if arg.type == 'array':
        return f'std::vector<{cpp_type(type("", (), {"type": arg.item_type}))}>'
    if arg.type == 'buffer':
        return f'std::vector<uint8_t>'
    if arg.type == 'string':
        return 'std::string'
    if arg.type == 'func':
        return 'std::string'
    if arg.type == 'map':
        return 'std::map<std::string, json>'
    if arg.type == 'any':
        return 'json'
    return arg.type

def cpp_types(args):
    return ', '.join(map(cpp_type, args))

def cpp_arg(arg):
    return f'{cpp_type(arg)} {arg.name}'

def cpp_args(args):
    return ', '.join(map(cpp_arg, args))

def cpp_rets(args):
    if len(args) == 0:
        return 'void'
    if len(args) == 1:
        return cpp_type(args[0])
    return f'std::tuple<{cpp_types(args)}>'
