#pragma once

#include <string>
#include <vector>
#include <boost/format.hpp>
#include <zmq.hpp>
#include <jsoncons/json.hpp>
#include <jsoncons_ext/cbor/cbor.hpp>
#include <functional>
#include <unordered_map>

using namespace jsoncons;

json bin(const char *s, int size = -1);
json bin(const uint8_t *b, int size);
json bin(const std::string &s);
json bin(const std::vector<uint8_t> &v);

#ifdef SIM_REMOTEAPICLIENT_OBJECTS
#include "RemoteAPIObjects.h"
#endif // SIM_REMOTEAPICLIENT_OBJECTS

class RemoteAPIClient
{
    using CallbackType = std::function<json(const json&)>;

public:
    RemoteAPIClient(const std::string host = "localhost", int rpcPort = 23000, int cntPort = -1, int verbose_ = -1);
    ~RemoteAPIClient();
    json call(const std::string &func, std::initializer_list<json> args);
    json call(const std::string &func, const json &args = json(json_array_arg));
    json getObject(const std::string &name);
    void require(const std::string &name);
    void setVerbose(int level = 1);
    void setStepping(bool enable = true); // for backw. comp., now via sim.setStepping
    void step(bool wait = true); // for backw. comp., now via sim.step
    void registerCallback(const std::string &funcName, CallbackType callback);

#ifdef SIM_REMOTEAPICLIENT_OBJECTS
    inline RemoteAPIObjects& getObject() { return remoteAPIObjects; }
private:
    RemoteAPIObjects remoteAPIObjects{this};
#endif // SIM_REMOTEAPICLIENT_OBJECTS

protected:
    void send(json &j);
    json recv();

private:
    CallbackType _getFunctionPointerByName(const std::string &funcName);
    int verbose{0};
    std::string uuid;
    int VERSION;
    zmq::context_t ctx;
    zmq::socket_t rpcSocket;
    std::unordered_map<std::string, CallbackType> callbacks;
};
