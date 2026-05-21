if __name__ == "__main__":
    import sys
    import argparse

    DESCRIPTION = """This Python script is designed to interact with CoppeliaSim (a robotics simulation platform) using its ZMQ Remote API.
The script reads a Lua script, executes it within CoppeliaSim, and saves the resulting constants or data to a JSON file."""
    parser = argparse.ArgumentParser(description=DESCRIPTION)
    parser.add_argument(
        "output_file", help="file path where the script will save the output data"
    )
    args = parser.parse_args()
    output_file = args.output_file

    try:
        import json

        from coppeliasim_zmqremoteapi_client import RemoteAPIClient

        client = RemoteAPIClient()

        with open(output_file, "wt") as f:
            with open(f"{__file__[:-3]}.lua", "rt") as g:
                client._send({"eval": g.read()})
            constants = client._process_response(client._recv())
            json.dump(constants, f, indent=4, separators=(",", ": "))
    except Exception as e:
        print(f"error: {e}")
        sys.exit(1)
