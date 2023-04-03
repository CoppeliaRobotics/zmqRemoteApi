# CoppeliaSim zmqRemoteApi C++ client

C++ client for the zmqRemoteApi.

## Usage example:

```cpp
#include "RemoteAPIClient.h"
#include <iostream>

int main()
{
    RemoteAPIClient client;
    auto sim = client.getObject().sim();

    int handle = sim.getObject("/Floor");

    return 0;
}
```
