#include <iostream>
#include <zmq.hpp>

int main()
{
    zmq::context_t context(1);
    zmq::socket_t socket(context, zmq::socket_type::push);
    socket.bind("tcp://*:5555");

    zmq::message_t message("Hello, world", 12);
    socket.send(message, zmq::send_flags::none);

    std::cout << "C++: Sent message: Hello, world" << std::endl;
    return 0;
}