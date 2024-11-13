#include <iostream>
#include <flatbuffers/flatbuffers.h>
#include <flatbuffers/verifier.h>
#include <fstream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include "monster_generated.h"

#define PORT 9999

int main()
{
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    server_fd = socket(AF_INET, SOCK_STREAM, 0);

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);
    bind(server_fd, (struct sockaddr *)&address, sizeof(address));

    listen(server_fd, 3);

    std::cout << "Waiting for connections..." << std::endl;
    new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen);

    flatbuffers::FlatBufferBuilder builder;

    // SEND MONSTER
    auto name = builder.CreateString("Orc");
    myGame::MonsterBuilder monsterBuilder(builder);
    monsterBuilder.add_id(1);
    monsterBuilder.add_name(name);
    monsterBuilder.add_hp(300);
    auto monster = monsterBuilder.Finish();

    builder.Finish(monster);

    uint8_t *buf = builder.GetBufferPointer();
    int size = builder.GetSize();

    send(new_socket, buf, size, 0);

    // RECEIVE MONSTER
    uint8_t buffer[1024];
    int bytes = recv(new_socket, buffer, sizeof(buffer), 0);

    if (bytes > 0)
    {
        std::cout << "Received data from client" << std::endl;

        flatbuffers::Verifier verifier(buffer, bytes);

        if (verifier.VerifyBuffer<myGame::Monster>(nullptr))
        {
            auto receivedMonster = flatbuffers::GetRoot<myGame::Monster>(buffer);
            std::cout << "Monster ID: " << receivedMonster->id() << std::endl;
            std::cout << "Monster Name: " << receivedMonster->name()->str() << std::endl;
            std::cout << "Monster HP: " << receivedMonster->hp() << std::endl;
        }
        else
        {
            std::cout << "Failed to verify the received data." << std::endl;
        }
    }

    close(new_socket);
    close(server_fd);

    return 0;
}