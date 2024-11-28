#include <iostream>
#include <cstring>
#include <netinet/in.h>
#include <unistd.h>
#include <flatbuffers/flatbuffers.h>
#include <robot/turtle_sim_generated.h>

#define PORT 12345

void sendFlatBuffer(int client_socket)
{
    // Serialize FlatBuffer data
    flatbuffers::FlatBufferBuilder builder;

    // Create turtle status with dummy values
    TurtleSim::TurtleStatusBuilder tsb(builder);
    tsb.add_x(1.0);
    tsb.add_y(2.0);
    tsb.add_theta(3.0);
    auto ts = tsb.Finish();

    builder.Finish(ts);

    uint8_t *buf = builder.GetBufferPointer();
    int size = builder.GetSize();

    // Send FlatBuffer data
    send(client_socket, buf, size, 0);
}

void handleClient(int client_socket)
{
    char buffer[1024];
    int bytes_received;

    // Simulate RTSP-like commands
    printf("Handling client\n");
    while ((bytes_received = recv(client_socket, buffer, sizeof(buffer), 0)) > 0)
    {
        std::string command(buffer, bytes_received);
        std::cout << "Received command: " << command << std::endl;

        // Pastikan ada terminasi baris baru
        if (command.find("SETUP") != std::string::npos)
        {
            // Setup command: Do any necessary initialization (if needed)
            std::string setup_response = "RTSP/1.0 200 OK\r\n";
            send(client_socket, setup_response.c_str(), setup_response.size(), 0);
        }
        else if (command.find("PLAY") != std::string::npos)
        {
            // PLAY command: Send FlatBuffer data
            sendFlatBuffer(client_socket);
        }
        else if (command.find("PAUSE") != std::string::npos)
        {
            // PAUSE command: Pause the stream (simulated, no action here)
            std::string pause_response = "RTSP/1.0 200 OK\r\n";
            send(client_socket, pause_response.c_str(), pause_response.size(), 0);
        }
        else if (command.find("TEARDOWN") != std::string::npos)
        {
            // TEARDOWN command: Close the session
            std::string teardown_response = "RTSP/1.0 200 OK\r\n";
            send(client_socket, teardown_response.c_str(), teardown_response.size(), 0);
            break; // Exit the loop after handling TEARDOWN
        }
        else
        {
            // Unknown command
            std::string error_response = "RTSP/1.0 400 Bad Request\r\n";
            send(client_socket, error_response.c_str(), error_response.size(), 0);
        }
    }

    close(client_socket);
}

int main()
{
    // Create a TCP socket
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == -1)
    {
        std::cerr << "Error creating socket" << std::endl;
        return -1;
    }

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        std::cerr << "Bind failed" << std::endl;
        return -1;
    }

    if (listen(server_fd, 3) < 0)
    {
        std::cerr << "Listen failed" << std::endl;
        return -1;
    }

    std::cout << "Waiting for connection..." << std::endl;

    int client_socket = accept(server_fd, NULL, NULL);
    if (client_socket < 0)
    {
        std::cerr << "Accept failed" << std::endl;
        return -1;
    }

    // Handle the client
    handleClient(client_socket);

    close(server_fd);
    return 0;
}
