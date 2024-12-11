#include <iostream>
#include <zmq.hpp>
#include <flatbuffers/flatbuffers.h>
#include "robot/turtle_sim_generated.h"

// int main()
// {
//     zmq::context_t context(1);
//     zmq::socket_t publisher(context, ZMQ_PUB);
//     publisher.bind("tcp://*:5555");

//     while (true)
//     {
//         float xTurtle = 1.0;
//         float yTurtle = 2.0;
//         float thetaTurtle = 3.0;

//         flatbuffers::FlatBufferBuilder builder;

//         TurtleSim::TurtleStatusBuilder tsb(builder);
//         tsb.add_x(xTurtle);
//         tsb.add_y(yTurtle);
//         tsb.add_theta(thetaTurtle);

//         auto ts = tsb.Finish();
//         builder.Finish(ts);

//         uint8_t *buf = builder.GetBufferPointer();
//         int size = builder.GetSize();

//         zmq::message_t topic_msg("status", 6);
//         zmq::message_t msg(buf, size);

//         publisher.send(topic_msg, zmq::send_flags::sndmore);
//         publisher.send(msg, zmq::send_flags::none);

//         float xTurtle2 = 4.0;
//         float yTurtle2 = 5.0;
//         float thetaTurtle2 = 6.0;

//         flatbuffers::FlatBufferBuilder builder2;

//         TurtleSim::TurtleStatusBuilder tsb2(builder2);
//         tsb2.add_x(xTurtle2);
//         tsb2.add_y(yTurtle2);
//         tsb2.add_theta(thetaTurtle2);

//         auto ts2 = tsb2.Finish();
//         builder2.Finish(ts2);

//         uint8_t *buf2 = builder2.GetBufferPointer();
//         int size2 = builder2.GetSize();

//         zmq::message_t topic_msg2("status2", 7);
//         zmq::message_t msg2(buf2, size2);

//         publisher.send(topic_msg2, zmq::send_flags::sndmore);
//         publisher.send(msg2, zmq::send_flags::none);

//         std::cout << "C++: Sent message on topic 'status'" << std::endl;
//     }
//     return 0;
// }

int main()
{
    zmq::context_t context(1);
    zmq::socket_t subscriber(context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:5556");
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "kitty cats", 10);

    while (true)
    {
        zmq::message_t sub_msg;
        zmq::message_t sub_topic;

        subscriber.recv(sub_topic, zmq::recv_flags::none);
        subscriber.recv(sub_msg, zmq::recv_flags::none);

        std::string topicStr(static_cast<char *>(sub_topic.data()), sub_topic.size());
        std::string messageStr(static_cast<char *>(sub_msg.data()), sub_msg.size());

        if (topicStr == "kitty cats")
        {
            std::cout << "Received: " << messageStr << std::endl;
        }
    }
    return 0;
}