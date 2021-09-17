#pragma once

#include <string>
#include <zmq.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

std::string str(const json &j);

json bin(const char *s, int size = -1);

class RemoteAPIClient
{
public:
    RemoteAPIClient(const std::string host = "localhost", const int port = 23000, bool verbose_ = false);
    json call(const std::string &func, const json &args = json::array());
    void setStepping(bool enable = true);
    void step();

protected:
    void send(const json &j);
    json recv();
    json callAddOn(const std::string &func, const json &args = json::array());

private:
    bool verbose{false};
    zmq::context_t ctx;
    zmq::socket_t sock;
};
