# ZMQ Remote API for CoppeliaSim

### Protocol

Connect a [`REQ`](https://zeromq.org/socket-api/#req-socket) socket to the endpoint (by default the ZMQ remote API server will listen to `tcp://*:23000`), send a message (see [request](#request) below), and read the response (see [response](#response) below). The requests/responses are encoded/decoded using [CBOR](https://cbor.io) format.

Example implementation for some languages are available in the [clients](tree/master/clients) directory.

#### Request

A request is an object with fields:
- `func` (string) the function name to call;
- `args` (array) the arguments to the function;

Example:

```json
{
    "func": "sim.getObject",
    "args": ["/Floor"]
}
```

#### Response

A response is an object with fields:
- `success` (boolean) `true` if the call succeeded, in which case the `ret` field will be set, or `false` if the call failed, in which case the `error` field will be set;
- `ret` (array) the return values of the function;
- `error` (string) the error message;

Example:

```json
{
    "success": true,
    "ret": [37]
}
```

In case of error, the exception message will be present:

```json
{
    "success": false,
    "error": "Object does not exist. (in function 'sim.getObject')"
}
```
