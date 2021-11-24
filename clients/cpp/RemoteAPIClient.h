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
    RemoteAPIClient(const std::string host = "localhost", int rpcPort = 23000, int cntPort = -1, int verbose_ = -1);
    json call(const std::string &func, std::initializer_list<json> args);
    json call(const std::string &func, const json &args = json(json_array_arg));
    json getObject(const std::string &name);
    void setVerbose(int level = 1);
    void setStepping(bool enable = true);
    void step(bool wait = true);

protected:
    long getStepCount(bool wait);
    void send(const json &j);
    json recv();

private:
    int verbose{0};
    std::string uuid;
    zmq::context_t ctx;
    zmq::socket_t rpcSocket;
    zmq::socket_t cntSocket;
};
