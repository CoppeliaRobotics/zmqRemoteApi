#include <iostream>
#include <string>
#include <vector>
#include <boost/format.hpp>
#include <zmq.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

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

    json call(const std::string &func, const json &args = json::array())
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
                throw std::runtime_error(resp["error"].get<std::string>());
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
            std::cout << "Sending: " << j.dump(4) << std::endl;

        std::vector<uint8_t> data = json::to_cbor(j);

        zmq::message_t msg(data.data(), data.size());
        sock.send(msg, zmq::send_flags::dontwait);
    }

    json recv()
    {
        zmq::message_t msg;
        sock.recv(msg);

        auto data = reinterpret_cast<const uint8_t*>(msg.data());
        json j = json::from_cbor(data, data + msg.size(), false);

        if(verbose)
            std::cout << "Received: " << j.dump(4) << std::endl;

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
    auto ret = client.call("sim.getObjectHandle", R"(["Floor"])"_json);
    std::cout << "Ret: " << ret << std::endl;
    return 0;
}
