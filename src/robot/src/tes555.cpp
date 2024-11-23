#include <liveMedia/liveMedia.hh>
#include <BasicUsageEnvironment/BasicUsageEnvironment.hh>
#include <groupsock/GroupsockHelper.hh>
#include <groupsock/Groupsock.hh>
#include <flatbuffers/flatbuffers.h>
#include "robot/turtle_sim_generated.h"

#define RTSP_PORT 8554

float xTurtle = 0, yTurtle = 0, thetaTurtle = 0;

// =============================
// RTSP Server
// =============================
UsageEnvironment *env;
RTSPServer *rtspServer;
TaskScheduler *scheduler;
RTPSink *rtpSink;

// =============================
// Function to Create FlatBuffer Data
// =============================
void sendFlatBufferData(void *clientData)
{
    flatbuffers::FlatBufferBuilder builder;

    TurtleSim::TurtleStatusBuilder tsb(builder);
    tsb.add_x(xTurtle);
    tsb.add_y(yTurtle);
    tsb.add_theta(thetaTurtle);
    auto turtleStatus = tsb.Finish();
    builder.Finish(turtleStatus);

    uint8_t *buffer = builder.GetBufferPointer();
    int bufferSize = builder.GetSize();

    // Stream data via RTSP
    if (rtpSink != nullptr)
    {
        rtpSink->sendData(buffer, bufferSize);
        printf("Sending FlatBuffer data: %d bytes\n", bufferSize);
    }
}

// =============================
// RTSP Server Setup
// =============================
void setupRTSPServer()
{
    scheduler = BasicTaskScheduler::createNew();
    env = BasicUsageEnvironment::createNew(*scheduler);
    rtspServer = RTSPServer::createNew(*env, RTSP_PORT);

    if (rtspServer == nullptr)
    {
        printf("Failed to create RTSP server.\n");
        return;
    }

    // Create a stream for sending FlatBuffer data
    rtpSink = new RTPSink(*env);

    // Schedule the sending of FlatBuffer data
    env->taskScheduler().scheduleDelayedTask(1000, (TaskFunc *)sendFlatBufferData, nullptr);

    printf("RTSP server is running on rtsp://127.0.0.1:%d\n", RTSP_PORT);
}

int main(int argc, char **argv)
{
    // Initialize RTSP Server
    setupRTSPServer();

    // Run RTSP server
    env->taskScheduler().doEventLoop();
    return 0;
}
