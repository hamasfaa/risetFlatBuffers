#include <ros/ros.h>
#include <robot/turtle_sim_generated.h>
#include <flatbuffers/flatbuffers.h>
#include <fstream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include "robot/PC2BS.h"
#include "robot/BS2PC.h"

#define PORT 1234

float xTurtle, yTurtle, thetaTurtle;
int new_socket;

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
    printf("xTurtle: %f, yTurtle: %f, thetaTurtle: %f\n", xTurtle, yTurtle, thetaTurtle);
    auto ts = tsb.Finish();

    builder.Finish(ts);

    uint8_t *buf = builder.GetBufferPointer();
    int size = builder.GetSize();

    send(new_socket, buf, size, 0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm");
    ros::NodeHandle nh;

    ros::Subscriber subPC2BS = nh.subscribe<robot::PC2BS>("pc2bs_topic", 1, pc2bsCallback);

    int server_fd;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    // =============================
    // SERVER
    // =============================
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);
    bind(server_fd, (struct sockaddr *)&address, sizeof(address));

    // =============================
    // CLIENT
    // =============================
    // otw

    listen(server_fd, 3);

    new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen);

    ros::Timer timer = nh.createTimer(ros::Duration(0.02), sendComm);

    ros::spin();

    close(new_socket);
    close(server_fd);

    return 0;
}