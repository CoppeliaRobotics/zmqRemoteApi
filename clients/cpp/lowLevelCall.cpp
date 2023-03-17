#include "RemoteAPIClient.h"
#include <iostream>
#include <iomanip>

int main()
{
    RemoteAPIClient client;
    auto sim = client.getObject().sim();

    /*
     * Function sim.getStringSignal doesn't play well with C++ static typing, as:
     *  - it can return no arguments
     *  - it can return a string or a byte buffer
     *
     * In such cases, one can directly use the low-level RemoteAPIClient::call() method.
     */
    json args(json_array_arg);
    args.push_back("testSignal");
    auto r = client.call("sim.getStringSignal", args);
    if(r.empty())
        std::cout << "signal is not set" << std::endl;
    else if(r[0].is_string())
        std::cout << "signal is a string: \"" << r[0].as<std::string>() << "\"" << std::endl;
    else if(r[0].is_byte_string())
        std::cout << "signal is a binary buffer of size " << r[0].as<std::vector<uint8_t>>().size() << std::endl;
    else
        std::cout << "signal is something else" << std::endl;
    /*
     * See also https://github.com/danielaparker/jsoncons/blob/master/doc/ref/basic_json.md
     */
    return 0;
}
