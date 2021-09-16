#include <iostream>
#include <string>
#include <vector>
#include <boost/format.hpp>
#include <zmq.hpp>
#include <jsoncons/json.hpp>
#include <jsoncons_ext/cbor/cbor.hpp>

using namespace jsoncons;

class RemoteAPIClient
{
public:
    RemoteAPIClient(const std::string host = "localhost", const int port = 23000, bool verbose_ = false)
        : sock(ctx, zmq::socket_type::req),
          verbose(verbose_)
    {
        auto addr = (boost::format("tcp://%s:%d") % host % port).str();
        sock.connect(addr);
    }

    json call(const std::string &func, const json &args = json(json_array_arg))
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
                throw std::runtime_error(resp["error"].as<std::string>());
            else
                throw std::runtime_error("unknown error");
        }
        const auto &ret = resp["ret"];
        return ret;
    }

protected:
    void send(const json &j)
    {
        if(verbose)
            std::cout << "Sending: " << pretty_print(j) << std::endl;

        std::vector<uint8_t> data;
        cbor::encode_cbor(j, data);

        zmq::message_t msg(data.data(), data.size());
        sock.send(msg, zmq::send_flags::dontwait);
    }

    json recv()
    {
        zmq::message_t msg;
        sock.recv(msg);

        auto data = reinterpret_cast<const uint8_t*>(msg.data());
        json j = cbor::decode_cbor<json>(data, data + msg.size());

        if(verbose)
            std::cout << "Received: " << pretty_print(j) << std::endl;

        return j;
    }

private:
    bool verbose{false};
    zmq::context_t ctx;
    zmq::socket_t sock;
};

int main()
{
    RemoteAPIClient client;
    auto ret = client.call("sim.getObjectHandle", json(json_array_arg, {"Floor"}));
    for(size_t i = 0; i < ret.size(); i++)
        std::cout << "ret[" << i << "]: " << ret[i] << std::endl;
    return 0;
}
