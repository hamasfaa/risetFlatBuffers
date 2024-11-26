// #include <iostream>
// #include <netinet/in.h>
// #include <sys/socket.h>
// #include <unistd.h>
// #include <string.h>

// #define PORT 554 // RTSP default port

// int rtsp_socket, client_sock;
// struct sockaddr_in server_addr, client_addr;
// socklen_t client_len = sizeof(client_addr);

// // Helper function to convert integer to string
// std::string intToString(int value)
// {
//     return std::to_string(value);
// }

// // RTSP Handlers
// void handleRTSPRequest(int client_sock)
// {
//     char buffer[1024];
//     int bytes_received = recv(client_sock, buffer, sizeof(buffer), 0);
//     if (bytes_received <= 0)
//         return;

//     // Print incoming request for debugging
//     std::cout << "[Server] Received request:\n"
//               << std::string(buffer, bytes_received) << std::endl;

//     // Handle DESCRIBE request (send 12345 to client)
//     if (strstr(buffer, "DESCRIBE") != NULL)
//     {
//         int data = 12345; // Example integer to send
//         std::string dataStr = intToString(data);

//         // Construct RTSP response with the data in the body
//         std::string response = "RTSP/1.0 200 OK\r\nCSeq: 1\r\n";
//         response += "Content-Type: text/plain\r\n";
//         response += "Content-Length: " + intToString(dataStr.length()) + "\r\n";
//         response += "\r\n" + dataStr; // Body with data

//         send(client_sock, response.c_str(), response.length(), 0);
//         std::cout << "[Server] Sent data: " << dataStr << std::endl;
//     }
//     // Handle client response (data sent back to server)
//     else if (strstr(buffer, "SETUP") != NULL)
//     {
//         std::string client_data(buffer);
//         std::cout << "[Server] Received from client: " << client_data << std::endl;

//         // Send acknowledgment back to client
//         std::string response = "RTSP/1.0 200 OK\r\nCSeq: 2\r\n";
//         response += "Content-Type: text/plain\r\n";
//         response += "Content-Length: 7\r\n";
//         response += "\r\nAcknowledged";

//         send(client_sock, response.c_str(), response.length(), 0);
//     }
//     else
//     {
//         // Handle other requests (e.g., SETUP, PLAY, etc.)
//         const char *error_response = "RTSP/1.0 405 Method Not Allowed\r\nCSeq: 1\r\n";
//         send(client_sock, error_response, strlen(error_response), 0);
//     }
// }

// int main()
// {
//     // Set up RTSP server socket
//     rtsp_socket = socket(AF_INET, SOCK_STREAM, 0);
//     if (rtsp_socket < 0)
//     {
//         perror("Socket creation failed");
//         return -1;
//     }

//     server_addr.sin_family = AF_INET;
//     server_addr.sin_addr.s_addr = INADDR_ANY;
//     server_addr.sin_port = htons(PORT);

//     if (bind(rtsp_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
//     {
//         perror("Bind failed");
//         return -1;
//     }

//     listen(rtsp_socket, 5);

//     std::cout << "[Server] Waiting for connection..." << std::endl;

//     while (true)
//     {
//         client_sock = accept(rtsp_socket, (struct sockaddr *)&client_addr, &client_len);
//         if (client_sock < 0)
//         {
//             perror("Client connection failed");
//             continue;
//         }

//         // Handle the RTSP request and send response
//         handleRTSPRequest(client_sock);

//         // Close client connection after response
//         close(client_sock);
//     }

//     close(rtsp_socket);
//     return 0;
// }
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>

#define PORT 554 // RTSP default port

int rtsp_socket, client_sock;
struct sockaddr_in server_addr, client_addr;
socklen_t client_len = sizeof(client_addr);

// RTSP Handlers
void handleRTSPRequest(int client_sock)
{
    char buffer[1024];
    int bytes_received = recv(client_sock, buffer, sizeof(buffer), 0);
    if (bytes_received <= 0)
        return;

    // Print incoming request for debugging
    std::cout << "[Server] Received request:\n"
              << std::string(buffer, bytes_received) << std::endl;

    // Handle incoming data (e.g., '54321')
    std::string received_data(buffer, bytes_received);

    // Print received data
    std::cout << "[Server] Received data: " << received_data << std::endl;

    // Send acknowledgment back to client
    std::string response = "RTSP/1.0 200 OK\r\nCSeq: 1\r\n";
    response += "Content-Type: text/plain\r\n";
    response += "Content-Length: 11\r\n"; // Length of the acknowledgment message
    response += "\r\nAcknowledged";

    send(client_sock, response.c_str(), response.length(), 0);
    std::cout << "[Server] Sent acknowledgment." << std::endl;
}

int main()
{
    // Set up RTSP server socket
    rtsp_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (rtsp_socket < 0)
    {
        perror("Socket creation failed");
        return -1;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT);

    if (bind(rtsp_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        perror("Bind failed");
        return -1;
    }

    listen(rtsp_socket, 5);

    std::cout << "[Server] Waiting for connection..." << std::endl;

    while (true)
    {
        client_sock = accept(rtsp_socket, (struct sockaddr *)&client_addr, &client_len);
        if (client_sock < 0)
        {
            perror("Client connection failed");
            continue;
        }

        // Handle the RTSP request and send response
        handleRTSPRequest(client_sock);

        // Close client connection after response
        close(client_sock);
    }

    close(rtsp_socket);
    return 0;
}
