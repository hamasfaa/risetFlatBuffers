#include <ros/ros.h>
#include <robot/turtle_sim_generated.h>
#include <flatbuffers/flatbuffers.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>
#include "robot/PC2BS.h"
#include "robot/BS2PC.h"

#define PORT 8554

float xTurtle, yTurtle, thetaTurtle;

ros::Publisher pubBS2PC;

int client_socket = -1;
ros::Timer timer;

bool isStreaming = false;

void pc2bsCallback(const robot::PC2BS::ConstPtr &msg)
{
    xTurtle = msg->x;
    yTurtle = msg->y;
    thetaTurtle = msg->theta;
    printf("DARI ROSSSSSSS xTurtle: %f, yTurtle: %f, thetaTurtle: %f\n", xTurtle, yTurtle, thetaTurtle);
}

void sendComm()
{
    if (client_socket == -1 || !isStreaming)
    {
        // Jika tidak ada koneksi atau streaming belum aktif, keluar
        return;
    }

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

    send(client_socket, buf, size, 0);
}

void handleClient()
{
    if (client_socket == -1)
    {
        ROS_WARN("Tidak ada koneksi client yang aktif.");
        return;
    }

    char buffer[1024];
    int bytes_received = recv(client_socket, buffer, sizeof(buffer), 0);

    if (bytes_received > 0)
    {
        std::string command(buffer, bytes_received);

        if (command.find("SETUP") != std::string::npos)
        {
            std::string setup_response = "RTSP/1.0 200 OK\r\n";
            send(client_socket, setup_response.c_str(), setup_response.size(), 0);
        }
        else if (command.find("PLAY") != std::string::npos)
        {
            isStreaming = true;
            std::string play_response = "RTSP/1.0 200 OK\r\n";
            send(client_socket, play_response.c_str(), play_response.size(), 0);
        }
        else if (command.find("PAUSE") != std::string::npos)
        {
            isStreaming = false;
            std::string pause_response = "RTSP/1.0 200 OK\r\n";
            send(client_socket, pause_response.c_str(), pause_response.size(), 0);
        }
        else if (command.find("TEARDOWN") != std::string::npos)
        {
            isStreaming = false;
            std::string teardown_response = "RTSP/1.0 200 OK\r\n";
            send(client_socket, teardown_response.c_str(), teardown_response.size(), 0);
            close(client_socket);
            client_socket = -1;
        }
    }
}

void startServer()
{
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == -1)
    {
        std::cerr << "Error creating socket" << std::endl;
        return;
    }

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        std::cerr << "Bind failed" << std::endl;
        return;
    }

    if (listen(server_fd, 3) < 0)
    {
        std::cerr << "Listen failed" << std::endl;
        return;
    }

    std::cout << "Waiting for connection..." << std::endl;

    client_socket = accept(server_fd, NULL, NULL);
    if (client_socket < 0)
    {
        std::cerr << "Accept failed" << std::endl;
        return;
    }

    std::cout << "Client connected." << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rtsp");
    ros::NodeHandle nh;

    pubBS2PC = nh.advertise<robot::BS2PC>("bs2pc_topic", 1);
    ros::Subscriber subPC2BS = nh.subscribe<robot::PC2BS>("pc2bs_topic", 1, pc2bsCallback);

    startServer();

    ros::Timer timer = nh.createTimer(ros::Duration(0.02), [&](const ros::TimerEvent &)
                                      { handleClient(); });

    ros::Timer send_timer = nh.createTimer(ros::Duration(0.02), [&](const ros::TimerEvent &)
                                           { sendComm(); });

    ros::spin();

    close(client_socket);
    return 0;
}
