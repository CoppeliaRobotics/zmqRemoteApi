def java_type(arg):
    def _(t):
        if t == 'array':
            return f'List<{_(arg.item_type)}>'
        if t == 'buffer':
            return 'byte[]'
        if t == 'bool':
            return 'Boolean'
        if t == 'int':
            return 'Long'
        if t == 'float':
            return 'Double'
        if t == 'string':
            return 'String'
        if t == 'func':
            return 'String'
        if t == 'map':
            return 'Map<String, Object>'
        if t == 'any':
            return 'Object'
        return t
    return _(arg.type)

def java_rets(args):
    if len(args) == 0:
        return 'void'
    if len(args) == 1:
        return java_type(args[0])
    return f'Object[]'
