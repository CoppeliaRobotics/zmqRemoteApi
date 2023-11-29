# ZMQ Remote API for CoppeliaSim

### Protocol

Connect a [`REQ`](https://zeromq.org/socket-api/#req-socket) socket to the endpoint (by default the ZMQ remote API server will listen to `tcp://*:23000`), send a message (see [request](#request) below), and read the response (see [response](#response) below). The requests/responses content is described in the next section using JSON notation, however the messages must be encoded/decoded using [CBOR](https://cbor.io).

#### Request

A request is an object with fields:
- `func` (string) the function name to call, or "_*executed*_" when a CoppeliaSim callback or command was executed;
- `args` (array) the arguments to the function, or the return values of a CoppeliaSim callback;
- `uuid` (string) the client's unique identifier;
- `ver` (int) the client's protocol version (currently 2). Can be omitted after the first contact;
- `lang` (string) the client's language. Not required;
- `timeout` (int) the client's timeout in seconds. Not required (default to ten minutes), in ignored after the first contact;
- `argsL` (int) an indication of args size. Useful since None, NULL, etc. are ignored by Lua;

Example:

```json
{
    "func": "sim.getObject",
    "args": ["/Floor"]
    "uuid": "c06b3832-5008-4cbb-b372-46ff92cacfe5",
    "ver": 2,
    "lang": "python",
    "timeout": 60,
    "argsL": 1,
}
```

#### Response

A response is an object with fields:
- `err` (string) the error message, in case of an error;
- `func` (string) a possible wait request ("_*wait*_") or callback function name to execute;
- `ret` (array) the return values of the function previously called, if none of above `err` or `func` field was returned;

Example:

```json
{
    "ret": [37]
}
```

In case of an error:

```json
{
    "err": "Object does not exist. (in function 'sim.getObject')"
}
```

## Versions

This table shows the protocol versions introduced with CoppeliaSim releases.

| Protocol version  | CoppeliaSim version |
| ----------------- | ------------------- |
| 1                 | 4.5.1 and earlier   |
| 2                 | 4.6.0 or later      |
