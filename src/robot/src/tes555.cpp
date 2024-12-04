#include <liveMedia.hh>
#include <BasicUsageEnvironment.hh>
#include <Groupsock.hh>
#include <RTSPServer.hh>
#include <iostream>
#include <string>

class DataSource : public FramedSource
{
public:
    DataSource(UsageEnvironment &env)
        : FramedSource(env), count(0), data("hello") {}

    static DataSource *createNew(UsageEnvironment &env)
    {
        return new DataSource(env);
    }

protected:
    virtual void doGetNextFrame() override
    {
        // Format data to send
        std::string message = data + std::to_string(count++);
        // Allocate memory for message
        unsigned frameSize = message.length();
        unsigned char *buffer = new unsigned char[frameSize];
        memcpy(buffer, message.c_str(), frameSize);

        // Set the data for the frame
        afterGetting(this);
        FramedSource::deliverFrame(buffer, frameSize);
    }

private:
    int count;
    std::string data;
};

void setupRTSPServer()
{
    TaskScheduler *scheduler = BasicTaskScheduler::createNew();
    UsageEnvironment *env = BasicUsageEnvironment::createNew(*scheduler);

    // Create RTSP server
    RTSPServer *rtspServer = RTSPServer::createNew(*env, 8554);
    if (rtspServer == nullptr)
    {
        std::cerr << "Failed to create RTSP server" << std::endl;
        return;
    }

    // Create the stream
    MediaSession *mediaSession = MediaSession::createNew(*env, "textstream", False);
    ServerMediaSubsession *mediaSubsession =
        ServerMediaSubsession::createNew(*env, DataSource::createNew(*env), "text/plain");

    // Add the stream to the RTSP server
    rtspServer->addServerMediaSession(mediaSession);

    // Start the server
    rtspServer->serveOnNewRTSPClient();

    // Enter event loop
    env->taskScheduler().doEventLoop();
}

int main()
{
    setupRTSPServer();
    return 0;
}
