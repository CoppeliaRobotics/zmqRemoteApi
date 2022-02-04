import os
import re
import sys
from pprint import PrettyPrinter
pp = PrettyPrinter(indent=4)

from calltip import CallTipParser

def color(i):
    return f'\033[{i:d}m' if sys.stdout.isatty() else ''

parser = CallTipParser()

def test(s):
    print(f'{s}\n')
    try:
        tree = parser.parse(s)
        print(f'Parse tree:\n\n{tree.pretty()}\n')
        func_def = parser.transform(tree)
        pp.pprint(func_def)
        return True
    except Exception as e:
        print(f'{color(91)}{e}{color(0)}\n')
        return False

if __name__ == '__main__':
    if sys.stdout.isatty():
        os.system("")  # enables ansi escape characters in terminal

    from zmqRemoteApi import RemoteAPIClient

    client = RemoteAPIClient()
    sim = client.getObject('sim')
    total, success = 0, 0
    funcList = sys.argv[1:] if len(sys.argv) > 1 else sim.getApiFunc(-1, '+')
    for fn in funcList:
        if fn in ('sim.test', ): continue
        if re.match(r'^sim\.', fn):
            total += 1
            print('-' * 78)
            print(f'{fn}...\n')
            s = sim.getApiInfo(-1, fn)
            s = s.splitlines()[0]
            if test(s):
                success += 1
    print(f'Could not parse {(1-success/total)*100:.1f}% of calltips')
