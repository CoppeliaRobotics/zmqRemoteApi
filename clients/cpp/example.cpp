#include "RemoteAPIClient.h"
#include <iostream>

int main()
{
    RemoteAPIClient client;

    std::cout << "sim.getObjectHandle(\"Floor\")..." << std::endl;
    auto ret = client.call("sim.getObjectHandle", json::array({"Floor"}));
    for(size_t i = 0; i < ret.size(); i++)
        std::cout << "ret[" << i << "]: " << ret[i] << std::endl;

    std::cout << "sim.getObjectAlias(" << ret[0] << ")..." << std::endl;
    ret = client.call("sim.getObjectAlias", json::array({ret[0]}));
    std::cout << "ret[0]: " << str(ret[0]) << std::endl;

    return 0;
}
