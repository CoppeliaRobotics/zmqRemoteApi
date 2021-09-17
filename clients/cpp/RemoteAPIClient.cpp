#include "RemoteAPIClient.h"
#include <iostream>
#include <vector>
#include <boost/format.hpp>

std::string str(const json &j)
{
    const auto &b = j.get_binary();
    return std::string{reinterpret_cast<const char*>(b.data()), b.size()};
}

json bin(const char *s, int size)
{
    if(size == -1) size = strlen(s);
    auto b = reinterpret_cast<const uint8_t *>(s);
    return json{std::vector<uint8_t>{b, b + size}};
}

RemoteAPIClient::RemoteAPIClient(const std::string host, const int port, bool verbose_)
    : sock(ctx, zmq::socket_type::req),
      verbose(verbose_)
{
    auto addr = (boost::format("tcp://%s:%d") % host % port).str();
    sock.connect(addr);
}

json RemoteAPIClient::call(const std::string &func, const json &args)
{
    json req;
    req["func"] = func;
    req["args"] = args;
    send(req);

    json resp = recv();
    bool ok = resp["success"].get<bool>();
    if(!ok)
    {
        if(resp.contains("error"))
            throw std::runtime_error(str(resp["error"]));
        else
            throw std::runtime_error("unknown error");
    }
    const auto &ret = resp["ret"];
    return ret;
}

void RemoteAPIClient::send(const json &j)
{
    if(verbose)
        std::cout << "Sending: " << j.dump(4) << std::endl;

    std::vector<uint8_t> data = json::to_cbor(j);

    zmq::message_t msg(data.data(), data.size());
    sock.send(msg, zmq::send_flags::dontwait);
}

json RemoteAPIClient::recv()
{
    zmq::message_t msg;
    sock.recv(msg);

    auto data = reinterpret_cast<const uint8_t*>(msg.data());
    json j = json::from_cbor(data, data + msg.size(), false);

    if(verbose)
        std::cout << "Received: " << j.dump(4) << std::endl;

    return j;
}
