#py from calltip import FuncDef
#py from cpp_utils import *
namespace RemoteAPIObject
{
#py for obj, func_defs in FuncDef.from_calltips_json(pycpp.params['calltips_json']).items():
    `obj`::`obj`(RemoteAPIClient *client)
        : _client(client)
    {
    }

#py for func, func_def in func_defs.items():
#py if func_def.in_args.is_variadic() or func_def.out_args.is_variadic():
#py continue
#py endif
    `cpp_rets(func_def.out_args)` `obj`::`func`(`cpp_args(func_def.in_args)`)
    {
        bool brk = false;
        json args(json_array_arg);
#py for arg in func_def.in_args:
#py wrap = (lambda x: f'bin({x})') if arg.type == 'buffer' else (lambda x: x)
#py if arg.is_optional():
        if(`arg.name`)
        {
            if(brk) throw std::runtime_error("no gaps allowed");
            else args.push_back(`wrap(f'*{arg.name}')`);
        }
        else brk = true;
#py else:
        args.push_back(`wrap(arg.name)`);
#py endif
#py endfor
        auto r = this->_client->call("`func_def.func_name`", args);
#py if len(func_def.out_args) == 1:
        return r[0].as<`cpp_type(func_def.out_args[0])`>();
#py elif len(func_def.out_args) > 1:
        return std::make_tuple(`', '.join(f'r[{i}].as<{cpp_type(arg)}>()' for i, arg in enumerate(func_def.out_args))`);
#py endif
    }

#py endfor
#py endfor
};
