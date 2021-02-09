#include <iostream>
#include <nlohmann/json.hpp>
#include <zmq.hpp>

using json = nlohmann::json;

int main()
{
    zmq::context_t ctx;
    zmq::socket_t sock(ctx, zmq::socket_type::req);
    sock.connect("tcp://localhost:23000");

    json jreq = R"({"func": "sim.getObjectHandle", "args": ["Floor"]})"_json;
    std::cout << "Sending: " << jreq.dump(4) << std::endl;
    std::vector<uint8_t> breq = json::to_cbor(jreq);
    zmq::message_t req(breq.begin(), breq.end());
    sock.send(req, zmq::send_flags::dontwait);

    zmq::message_t resp;
    sock.recv(resp);
    auto data = reinterpret_cast<const uint8_t*>(resp.data());
    json jresp = json::from_cbor(data, data + resp.size(), false);
    std::cout << "Received: " << jresp.dump(4) << std::endl;

    return 0;
}
