#py from calltip import FuncDef
#py from cpp_utils import *
class RemoteAPIClient;

namespace RemoteAPIObject
{
#py for obj, func_defs in FuncDef.from_calltips_json(pycpp.params['calltips_json']).items():
    class `obj`
    {
    protected:
        RemoteAPIClient *_client;
    public:
        `obj`(RemoteAPIClient *client);

#py for func, func_def in func_defs.items():
#py if func_def.in_args.is_variadic() or func_def.out_args.is_variadic():
#py continue
#py endif
    `cpp_rets(func_def.out_args)` `func`(`cpp_args(func_def.in_args)`);
#py endfor
    };

#py endfor
};
