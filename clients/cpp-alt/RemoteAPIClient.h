#pragma once

#include <string>
#include <vector>
#include <boost/format.hpp>
#include <zmq.hpp>
#include <jsoncons/json.hpp>
#include <jsoncons_ext/cbor/cbor.hpp>

using namespace jsoncons;

std::string str(const json& j);

json bin(const char *s, int size = -1);

class RemoteAPIClient
{
public:
    RemoteAPIClient(const std::string host = "localhost", const int port = 23000, bool verbose_ = false);
    json call(const std::string &func, const json &args = json(json_array_arg));

protected:
    void send(const json &j);
    json recv();

private:
    bool verbose{false};
    zmq::context_t ctx;
    zmq::socket_t sock;
};
