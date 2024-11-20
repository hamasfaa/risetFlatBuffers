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
int new_socket;
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

    send(new_socket, buf, size, 0);
}

void recvComm(const ros::TimerEvent &)
{
    uint8_t buffer[1024];
    int bytesReceived = recv(new_socket, buffer, sizeof(buffer), MSG_DONTWAIT);

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

    int server_fd;
    struct sockaddr_in addr;
    int opt = 1;
    int addrlen = sizeof(addr);

    // =============================
    // SERVER & CLIENT
    // =============================
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(PORT);
    bind(server_fd, (struct sockaddr *)&addr, sizeof(addr));
    listen(server_fd, 3);

    printf("[C++] Waiting for client connection...\n");
    new_socket = accept(server_fd, (struct sockaddr *)&addr, (socklen_t *)&addrlen);
    printf("[C++] Client connected.\n");

    ros::Timer timer = nh.createTimer(ros::Duration(0.02), sendComm);
    ros::Timer timer2 = nh.createTimer(ros::Duration(0.02), recvComm);

    ros::spin();

    close(new_socket);
    close(server_fd);

    return 0;
}