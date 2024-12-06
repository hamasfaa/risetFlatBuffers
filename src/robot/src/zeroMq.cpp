#include <iostream>
#include <zmq.hpp>
#include <flatbuffers/flatbuffers.h>
#include "robot/turtle_sim_generated.h"

int main()
{
    zmq::context_t context(1);
    zmq::socket_t socket(context, zmq::socket_type::push);
    socket.bind("tcp://*:5555");

    float xTurtle = 1.0;
    float yTurtle = 2.0;
    float thetaTurtle = 3.0;

    flatbuffers::FlatBufferBuilder builder;

    TurtleSim::TurtleStatusBuilder tsb(builder);
    tsb.add_x(xTurtle);
    tsb.add_y(yTurtle);
    tsb.add_theta(thetaTurtle);

    auto ts = tsb.Finish();

    builder.Finish(ts);

    uint8_t *buf = builder.GetBufferPointer();
    int size = builder.GetSize();

    zmq::message_t message(buf, size);

    socket.send(message, zmq::send_flags::none);

    std::cout << "C++: Sent message: Hello, world" << std::endl;
    return 0;
}