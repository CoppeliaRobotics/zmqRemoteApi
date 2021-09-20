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
json bin(const uint8_t *b, int size);
json bin(const std::string &s);
json bin(const std::vector<uint8_t> &v);

class RemoteAPIClient
{
public:
    RemoteAPIClient(const std::string host = "localhost", const int port = 23000, bool verbose_ = false);
    json call(const std::string &func, std::initializer_list<json> args);
    json call(const std::string &func, const json &args = json(json_array_arg));
    json getObject(const std::string &name);
    void setStepping(bool enable = true);
    void step();

protected:
    void send(const json &j);
    json recv();
    json callAddOn(const std::string &func, const json &args = json(json_array_arg));

private:
    bool verbose{false};
    zmq::context_t ctx;
    zmq::socket_t sock;
};
