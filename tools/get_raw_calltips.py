if __name__ == '__main__':
    import sys

    if len(sys.argv) != 2:
        print(f'usage: {sys.argv[0]} <output-file.json>')
        sys.exit(2)

    try:
        import json

        from coppeliasim_zmqremoteapi_client import RemoteAPIClient

        client = RemoteAPIClient()
        sim = client.require('sim')

        def progress(i, lst, prev_pct):
            pct = int((i + 1) * 100 / len(lst))
            if pct == prev_pct:
                return pct
            msg = f'\rReading calltips: {pct}%...'
            if pct == 100:
                msg = '\r' + ' ' * len(msg) + '\r'
            sys.stdout.write(msg)
            sys.stdout.flush()
            return pct

        with open(sys.argv[1], 'wt') as f:
            all_funcs = sim.getApiFunc(-1, '+')
            calltips = {}
            pct = -1
            for i, func in enumerate(all_funcs):
                pct = progress(i, all_funcs, pct)
                calltips[func] = sim.getApiInfo(-1, func)
            json.dump(calltips, f, indent=4, separators=(',', ': '))
    except Exception as e:
        print(f'error: {e}')
        sys.exit(1)
