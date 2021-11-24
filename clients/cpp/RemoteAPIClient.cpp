#include "RemoteAPIClient.h"
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <boost/format.hpp>
#include <boost/optional.hpp>
#include <random>
#include <sstream>

namespace uuid {
    static std::random_device              rd;
    static std::mt19937                    gen(rd());
    static std::uniform_int_distribution<> dis(0, 15);
    static std::uniform_int_distribution<> dis2(8, 11);

    std::string generate_uuid_v4() {
        std::stringstream ss;
        int i;
        ss << std::hex;
        for (i = 0; i < 8; i++) {
            ss << dis(gen);
        }
        ss << "-";
        for (i = 0; i < 4; i++) {
            ss << dis(gen);
        }
        ss << "-4";
        for (i = 0; i < 3; i++) {
            ss << dis(gen);
        }
        ss << "-";
        ss << dis2(gen);
        for (i = 0; i < 3; i++) {
            ss << dis(gen);
        }
        ss << "-";
        for (i = 0; i < 12; i++) {
            ss << dis(gen);
        };
        return ss.str();
    }
}

std::string str(const json& j)
{
    const auto &v = j.as< std::vector<uint8_t> >(byte_string_arg, semantic_tag::base64);
    return std::string{v.begin(), v.end()};
}

json bin(const char *s, int size)
{
    if(size == -1) size = strlen(s);
    return bin(reinterpret_cast<const uint8_t *>(s), size);
}

json bin(const uint8_t *b, int size)
{
    return bin(std::vector<uint8_t>{b, b + size});
}

json bin(const std::string &s)
{
    return bin(s.data(), s.length());
}

json bin(const std::vector<uint8_t> &v)
{
    return json{byte_string_arg, v};
}

RemoteAPIClient::RemoteAPIClient(const std::string host, int rpcPort, int cntPort, int verbose_)
    : rpcSocket(ctx, zmq::socket_type::req),
      cntSocket(ctx, zmq::socket_type::sub),
      verbose(verbose_)
{
    if(verbose == -1)
    {
        if(const char* verboseStr = std::getenv("VERBOSE"))
            verbose = std::atoi(verboseStr);
        else
            verbose = 0;
    }

    if(cntPort == -1)
        cntPort = rpcPort + 1;
    
    uuid = uuid::generate_uuid_v4();

    auto rpcAddr = (boost::format("tcp://%s:%d") % host % rpcPort).str();
    rpcSocket.connect(rpcAddr);

    auto cntAddr = (boost::format("tcp://%s:%d") % host % cntPort).str();
    cntSocket.set(zmq::sockopt::subscribe, "");
    cntSocket.set(zmq::sockopt::conflate, 1);
    cntSocket.connect(cntAddr);
}

json RemoteAPIClient::call(const std::string &func, std::initializer_list<json> args)
{
    return call(func, json::make_array(args));
}

json RemoteAPIClient::call(const std::string &func, const json &args)
{
    json req;
    req["func"] = func;
    req["args"] = args;
    send(req);

    json resp = recv();
    bool ok = resp["success"].as<bool>();
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

json RemoteAPIClient::getObject(const std::string &name)
{
    return call("zmqRemoteApi.info", json(json_array_arg, {name}));
}

void RemoteAPIClient::setVerbose(int level)
{
    verbose = level;
}

void RemoteAPIClient::setStepping(bool enable)
{
    call("setStepping", {enable,uuid});
}

void RemoteAPIClient::step(bool wait)
{
    if(wait) getStepCount(false);
    call("step",{uuid});
    if(wait) getStepCount(true);
}

long RemoteAPIClient::getStepCount(bool wait)
{
    zmq::message_t msg;
    if(!cntSocket.recv(msg, wait ? zmq::recv_flags::none : zmq::recv_flags::dontwait))
        return -1;

    auto c = reinterpret_cast<const int*>(msg.data())[0];

    if(verbose > 0)
        std::cout << "Step count: " << c << std::endl;

    return c;
}

void RemoteAPIClient::send(const json &j)
{
    if(verbose > 0)
        std::cout << "Sending: " << pretty_print(j) << std::endl;

    std::vector<uint8_t> data;
    cbor::encode_cbor(j, data);

    if(verbose > 1)
    {
        std::cout << "Sending (raw):";
        for(size_t i = 0; i < data.size(); i++)
            std::cout << " " << std::hex << std::setw(2) << std::setfill('0') << int(data[i]);
        std::cout << std::endl;
    }

    zmq::message_t msg(data.data(), data.size());
    rpcSocket.send(msg, zmq::send_flags::dontwait);
}

json RemoteAPIClient::recv()
{
    zmq::message_t msg;
    rpcSocket.recv(msg);

    auto data = reinterpret_cast<const uint8_t*>(msg.data());

    if(verbose > 1)
    {
        std::cout << "Received (raw):";
        for(size_t i = 0; i < msg.size(); i++)
            std::cout << " " << std::hex << std::setw(2) << std::setfill('0') << int(data[i]);
        std::cout << std::endl;
    }

    json j = cbor::decode_cbor<json>(data, data + msg.size());

    if(verbose > 0)
        std::cout << "Received: " << pretty_print(j) << std::endl;

    return j;
}
