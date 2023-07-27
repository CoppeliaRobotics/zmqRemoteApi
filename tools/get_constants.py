if __name__ == '__main__':
    import sys

    if len(sys.argv) != 2:
        print(f'usage: {sys.argv[0]} <output-file.json>')
        sys.exit(2)

    try:
        import json

        from coppeliasim_zmqremoteapi_client import RemoteAPIClient

        client = RemoteAPIClient()

        with open(sys.argv[1], 'wt') as f:
            with open(f'{__file__[:-3]}.lua', 'rt') as g:
                client._send({'eval': g.read()})
            constants = client._process_response(client._recv())
            json.dump(constants, f, indent=4, separators=(',', ': '))
    except Exception as e:
        print(f'error: {e}')
        sys.exit(1)
