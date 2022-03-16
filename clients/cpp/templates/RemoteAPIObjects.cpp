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
        auto r = this->_client->call("`func_def.func_name`", {`', '.join(a.name for a in func_def.in_args)`});
#py if len(func_def.out_args) == 1:
        return r[0].as<`cpp_type(func_def.out_args[0])`>();
#py elif len(func_def.out_args) > 1:
        return std::make_tuple(`', '.join(f'r[{i}].as<{cpp_type(a)}>()' for i, a in enumerate(func_def.out_args))`);
#py endif
    }

#py endfor
#py endfor
};
