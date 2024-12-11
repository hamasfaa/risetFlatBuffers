#include <ros/ros.h>
#include <robot/turtle_sim_generated.h>
#include <zmq.hpp>
#include <flatbuffers/flatbuffers.h>
#include <stdio.h>
#include <unistd.h>
#include "robot/PC2BS.h"
#include "robot/BS2PC.h"

float xTurtle, yTurtle, thetaTurtle;

ros::Publisher pubBS2PC;

zmq::context_t context(1);
zmq::socket_t publisher(context, ZMQ_PUB);
zmq::socket_t subscriber(context, ZMQ_SUB);

void pc2bsCallback(const robot::PC2BS::ConstPtr &msg)
{
    xTurtle = msg->x;
    yTurtle = msg->y;
    thetaTurtle = msg->theta;
    // printf("DARI ROSSSSSSS xTurtle: %f, yTurtle: %f, thetaTurtle: %f\n", xTurtle, yTurtle, thetaTurtle);
}

void sendComm(const ros::TimerEvent &)
{
    flatbuffers::FlatBufferBuilder builder;

    TurtleSim::TurtleStatusBuilder tsb(builder);
    tsb.add_x(xTurtle);
    tsb.add_y(yTurtle);
    tsb.add_theta(thetaTurtle);
    auto ts = tsb.Finish();

    builder.Finish(ts);

    uint8_t *buf = builder.GetBufferPointer();
    int size = builder.GetSize();

    zmq::message_t topic_msg("status", 6);
    zmq::message_t msg(buf, size);

    publisher.send(topic_msg, zmq::send_flags::sndmore);
    publisher.send(msg, zmq::send_flags::none);
}

void recvComm(const ros::TimerEvent &)
{
    zmq::message_t sub_msg;
    zmq::message_t sub_topic;

    subscriber.recv(sub_topic, zmq::recv_flags::none);
    subscriber.recv(sub_msg, zmq::recv_flags::none);

    std::string topicStr(static_cast<char *>(topic.data()), topic.size());

    if (topicStr == "control")
    {
        uint8_t *buffer = static_cast<uint8_t *>(sub_msg.data());
        int bytesReceived = sub_msg.size();

        flatbuffers::Verifier verifier(buffer, bytesReceived);
        auto cc = flatbuffers::GetRoot<TurtleSim::ControlCommand>(buffer);

        float linear_velocity = cc->linear_velocity();
        float angular_velocity = cc->angular_velocity();

        printf("[C++] Received: linear_velocity: %f, angular_velocity: %f\n", linear_velocity, angular_velocity);

        robot::BS2PC msg;
        msg.linear_velocity = linear_velocity;
        msg.angular_velocity = angular_velocity;
        pubBS2PC.publish(msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm");
    ros::NodeHandle nh;

    publisher.bind("tcp://*:5555");
    subscriber.connect("tcp://localhost:5555");
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "control", 7);

    pubBS2PC = nh.advertise<robot::BS2PC>("bs2pc_topic", 1);
    ros::Subscriber subPC2BS = nh.subscribe<robot::PC2BS>("pc2bs_topic", 1, pc2bsCallback);

    ros::Timer timer2 = nh.createTimer(ros::Duration(0.02), sendComm);
    // ros::Timer timer2 = nh.createTimer(ros::Duration(0.02), recvComm);

    ros::spin();

    return 0;
}