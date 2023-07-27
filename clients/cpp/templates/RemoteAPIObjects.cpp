#py from calltip import FuncDef
#py from cpp_utils import *
#py all_func_defs = FuncDef.from_calltips_json(pycpp.params['calltips_json'], pycpp.params['include_objects'], pycpp.params['exclude_objects'], pycpp.params['exclude_methods'])
namespace RemoteAPIObject
{
#py for obj, func_defs in all_func_defs.items():
    `obj`::`obj`(RemoteAPIClient *client)
        : _client(client)
    {
        _client->require("`obj`");
    }

#py if obj == 'sim':
#include "sim-deprecated.cpp"
#include "sim-special.cpp"

#py endif
#py for func, func_def in func_defs.items():
#py if func_def.in_args.is_variadic() or func_def.out_args.is_variadic():
#py continue
#py endif
    `cpp_rets(func_def.out_args)` `obj`::`func`(`cpp_args(func_def.in_args)`)
    {
        bool _brk = false;
        json _args(json_array_arg);
#py for arg in func_def.in_args:
#py wrap = (lambda x: f'bin({x})') if arg.type == 'buffer' else (lambda x: x)
#py if arg.is_optional():
        if(`arg.name`)
        {
            if(_brk) throw std::runtime_error("no gaps allowed");
            else _args.push_back(`wrap(f'*{arg.name}')`);
        }
        else _brk = true;
#py else:
        _args.push_back(`wrap(arg.name)`);
#py endif
#py endfor
        auto _ret = this->_client->call("`func_def.func_name`", _args);
#py if len(func_def.out_args) == 1:
        return _ret[0].as<`cpp_type(func_def.out_args[0])`>();
#py elif len(func_def.out_args) > 1:
        return std::make_tuple(`', '.join(f'_ret[{i}].as<{cpp_type(arg)}>()' for i, arg in enumerate(func_def.out_args))`);
#py endif
    }

#py endfor
#py endfor
};

RemoteAPIObjects::RemoteAPIObjects(RemoteAPIClient *client)
{
    this->client = client;
}

#py for obj, func_defs in all_func_defs.items():
RemoteAPIObject::`obj` RemoteAPIObjects::`obj`()
{
    return RemoteAPIObject::`obj`(this->client);
}

#py endfor
