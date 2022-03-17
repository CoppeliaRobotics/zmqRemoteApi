#py from calltip import FuncDef
#py from cpp_utils import *
#py import json
#py all_func_defs = FuncDef.from_calltips_json(pycpp.params['calltips_json'])
#py f = open(pycpp.params['constants_json'], 'rt')
#py all_constants = json.load(f)
#py f.close()
class RemoteAPIClient;

namespace RemoteAPIObject
{
#py for obj, func_defs in all_func_defs.items():
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
        `cpp_rets(func_def.out_args)` `func`(`cpp_args_d(func_def.in_args)`);
#py endfor

#py for k, v in sorted(all_constants.get(obj, {}).items()):
#py if isinstance(v, dict):
        struct _`k` {
#py for k1, v1 in sorted(v.items()):
            const int `k1` = `v1`;
#py endfor
        };
        const _`k` `k`;
#py else:
        const int `k` = `v`;
#py endif
#py endfor
    };

#py endfor
};

class RemoteAPIObjects
{
public:
    RemoteAPIObjects(RemoteAPIClient *client);
#py for obj, func_defs in all_func_defs.items():
    RemoteAPIObject::`obj` `obj`();
#py endfor
private:
    RemoteAPIClient *client;
};
