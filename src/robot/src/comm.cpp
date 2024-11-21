#include <ros/ros.h>
#include <robot/turtle_sim_generated.h>
#include <flatbuffers/flatbuffers.h>
#include <fstream>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>
#include "robot/PC2BS.h"
#include "robot/BS2PC.h"

#define PORT 1234

float xTurtle, yTurtle, thetaTurtle;

int udp_socket;

// =============================
// CLIENT
// =============================
struct sockaddr_in client_addr;
socklen_t client_len = sizeof(client_addr);

// =============================
// SERVER
// =============================
struct sockaddr_in server_addr;

ros::Publisher pubBS2PC;

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
    // printf("[C++] Sending: x: %f, y: %f, theta: %f\n", xTurtle, yTurtle, thetaTurtle);
    auto ts = tsb.Finish();

    builder.Finish(ts);

    uint8_t *buf = builder.GetBufferPointer();
    int size = builder.GetSize();

    sendto(udp_socket, buf, size, 0, (struct sockaddr *)&client_addr, client_len);
}

void recvComm(const ros::TimerEvent &)
{
    uint8_t buffer[1024];
    int bytesReceived = recvfrom(udp_socket, buffer, sizeof(buffer), MSG_DONTWAIT, (struct sockaddr *)&client_addr, &client_len);

    if (bytesReceived > 0)
    {
        flatbuffers::Verifier verifier(buffer, bytesReceived);

        if (verifier.VerifyBuffer<TurtleSim::ControlCommand>(nullptr))
        {
            auto cc = flatbuffers::GetRoot<TurtleSim::ControlCommand>(buffer);

            float linear_velocity = cc->linear_velocity();
            float angular_velocity = cc->angular_velocity();

            printf("[C++] Received: linear_velocity: %f, angular_velocity: %f\n", linear_velocity, angular_velocity);

            robot::BS2PC msg;
            msg.linear_velocity = linear_velocity;
            msg.angular_velocity = angular_velocity;
            pubBS2PC.publish(msg);
        }
        else
        {
            printf("[C++] Invalid buffer.\n");
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm");
    ros::NodeHandle nh;

    pubBS2PC = nh.advertise<robot::BS2PC>("bs2pc_topic", 1);
    ros::Subscriber subPC2BS = nh.subscribe<robot::PC2BS>("pc2bs_topic", 1, pc2bsCallback);

    udp_socket = socket(AF_INET, SOCK_DGRAM, 0);

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT);

    bind(udp_socket, (struct sockaddr *)&server_addr, sizeof(server_addr));

    ros::Timer timer = nh.createTimer(ros::Duration(0.02), sendComm);
    ros::Timer timer2 = nh.createTimer(ros::Duration(0.02), recvComm);

    ros::spin();

    close(udp_socket);

    return 0;
}