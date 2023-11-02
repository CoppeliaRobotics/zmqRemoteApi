#py from calltip import FuncDef
#py from java_utils import *
#py import json
#py all_func_defs = FuncDef.from_calltips_json(pycpp.params['calltips_json'], pycpp.params['include_objects'], pycpp.params['exclude_objects'], pycpp.params['exclude_methods'])
#py f = open(pycpp.params['constants_json'], 'rt')
#py all_constants = json.load(f)
#py f.close()
#py reservedNames = {'abstract', 'assert', 'boolean', 'break', 'byte', 'case', 'catch', 'char', 'class', 'continue', 'const', 'default', 'do', 'double', 'else', 'enum', 'exports', 'extends', 'final', 'finally', 'float', 'for', 'goto', 'if', 'implements', 'import', 'instanceof', 'int', 'interface', 'long', 'module', 'native', 'new', 'package', 'private', 'protected', 'public', 'requires', 'return', 'short', 'static', 'strictfp', 'super', 'switch', 'synchronized', 'this', 'throw', 'throws', 'transient', 'try', 'var', 'void', 'volatile', 'while'}
#py fixReserved = lambda n: n + ('_' if n in reservedNames else '')
package com.coppeliarobotics.remoteapi.zmq;

import java.util.*;

import co.nstant.in.cbor.*;

public class RemoteAPIObjects
{
    private final RemoteAPIClient client;

    public RemoteAPIObjects(RemoteAPIClient client)
    {
        this.client = client;
    }

#py for obj, func_defs in all_func_defs.items():
    public class _`obj` extends `f"com.coppeliarobotics.remoteapi.zmq.objects.special._{obj}" if obj in ('sim', ) else "RemoteAPIObject"`
    {
        public _`obj`(RemoteAPIClient client)
        {
            super(client);
        }

#py for func, func_def in func_defs.items():
#py if func_def.in_args.is_variadic() or func_def.out_args.is_variadic():
#py continue
#py endif
        public `java_rets(func_def.out_args)` `func`(Object... args) throws CborException
        {
#py if len(func_def.out_args) == 0:
            this.client.call("`obj`.`func`", args);
#py else:
            Object[] rets = this.client.call("`obj`.`func`", args);
#py if len(func_def.out_args) > 1:
            return rets;
#py else:
            return ((`java_type(func_def.out_args[0])`)rets[0]);
#py endif
#py endif
        }

#py endfor
#py for k, v in sorted(all_constants.get(obj, {}).items()):
#py if isinstance(v, dict):
        class _`k` {
#py for k1, v1 in sorted(v.items()):
            public static final int `fixReserved(k1)` = `v1`;
#py endfor
        }

        public static final _`k` `fixReserved(k)`;
#py else:
        public static final int `fixReserved(k)` = `v`;
#py endif
#py endfor
    }

    public _`obj` `obj`() {
        return new _`obj`(this.client);
    }

#py endfor
}
