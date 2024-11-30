#include <gst/gst.h>
#include <flatbuffers/flatbuffers.h>
#include <robot/turtle_sim_generated.h>

void send_flatbuffer_data(float xTurtle, float yTurtle, float thetaTurtle)
{
    // Initialize GStreamer
    gst_init(nullptr, nullptr);

    // Create pipeline
    GstElement *pipeline = gst_pipeline_new("udp-pipeline");
    GstElement *appsrc = gst_element_factory_make("appsrc", "appsrc");
    GstElement *udpsink = gst_element_factory_make("udpsink", "udpsink");

    if (!pipeline || !appsrc || !udpsink)
    {
        g_printerr("Failed to create GStreamer elements.\n");
        return;
    }

    // Set udpsink properties
    g_object_set(udpsink, "host", "127.0.0.1", "port", 5000, nullptr);

    // Add elements to pipeline
    gst_bin_add_many(GST_BIN(pipeline), appsrc, udpsink, nullptr);
    if (!gst_element_link(appsrc, udpsink))
    {
        g_printerr("Failed to link elements.\n");
        gst_object_unref(pipeline);
        return;
    }

    // Create FlatBuffer data
    flatbuffers::FlatBufferBuilder builder;
    TurtleSim::TurtleStatusBuilder tsb(builder);
    tsb.add_x(xTurtle);
    tsb.add_y(yTurtle);
    tsb.add_theta(thetaTurtle);
    auto ts = tsb.Finish();
    builder.Finish(ts);

    // Get pointer to serialized buffer
    uint8_t *buf = builder.GetBufferPointer();
    size_t size = builder.GetSize();

    // Create a GStreamer buffer
    GstBuffer *gst_buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
    gst_buffer_fill(gst_buffer, 0, buf, size);

    // Push buffer to appsrc
    GstFlowReturn ret;
    g_signal_emit_by_name(appsrc, "push-buffer", gst_buffer, &ret);

    if (ret != GST_FLOW_OK)
    {
        g_printerr("Failed to push buffer to appsrc.\n");
    }

    // Clean up
    gst_buffer_unref(gst_buffer);
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    // Wait to let the data be sent
    g_usleep(1000000); // Sleep for 1 second

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
}

int main()
{
    send_flatbuffer_data(1.0, 2.0, 3.14159);
    return 0;
}
