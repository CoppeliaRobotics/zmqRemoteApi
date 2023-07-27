#py from calltip import FuncDef
#py from cpp_utils import *
#py import json
#py all_func_defs = FuncDef.from_calltips_json(pycpp.params['calltips_json'], pycpp.params['include_objects'], pycpp.params['exclude_objects'], pycpp.params['exclude_methods'])
#py f = open(pycpp.params['constants_json'], 'rt')
#py all_constants = json.load(f)
#py f.close()
#py reservedNames = {'union','auto'}
#py fixReserved = lambda n: n + ('_' if n in reservedNames else '')

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

#py if obj == 'sim':
#include "sim-deprecated.h"
#include "sim-special.h"

#py endif
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
#ifndef `k1`
            const int `fixReserved(k1)` = `v1`;
#endif
#py endfor
        };
#ifndef `k`
        const _`k` `fixReserved(k)`;
#endif
#py else:
#ifndef `k`
        const int `fixReserved(k)` = `v`;
#endif
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
