#include <gst/gst.h>
#include <flatbuffers/flatbuffers.h>
#include <robot/turtle_sim_generated.h>
#include <iostream>

static gboolean on_message(GstBus *bus, GstMessage *msg, gpointer user_data)
{
    switch (GST_MESSAGE_TYPE(msg))
    {
    case GST_MESSAGE_ERROR:
    {
        GError *err;
        gchar *debug_info;
        gst_message_parse_error(msg, &err, &debug_info);
        g_printerr("Error received from element %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
        g_printerr("Debugging information: %s\n", debug_info ? debug_info : "none");
        g_clear_error(&err);
        g_free(debug_info);
        break;
    }
    default:
        break;
    }
    return TRUE;
}

static GstFlowReturn on_new_sample(GstElement *sink, gpointer user_data)
{
    GstSample *sample = nullptr;
    g_signal_emit_by_name(sink, "pull-sample", &sample);
    if (sample)
    {
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        GstMapInfo map;
        if (gst_buffer_map(buffer, &map, GST_MAP_READ))
        {
            // Parse FlatBuffer data
            auto buf = map.data;
            auto size = map.size;

            // Parse FlatBuffer message
            flatbuffers::Verifier verifier(buf, size);
            if (!verifier.VerifyBuffer<TurtleSim::TurtleStatus>(nullptr))
            {
                g_printerr("Received invalid FlatBuffer data.\n");
            }
            else
            {
                auto turtle_status = flatbuffers::GetRoot<TurtleSim::TurtleStatus>(buf);
                float x = turtle_status->x();
                float y = turtle_status->y();
                float theta = turtle_status->theta();

                std::cout << "Received Turtle Status: x = " << x << ", y = " << y << ", theta = " << theta << std::endl;
            }
            gst_buffer_unmap(buffer, &map);
        }
        gst_sample_unref(sample);
    }
    return GST_FLOW_OK;
}

int main(int argc, char *argv[])
{
    gst_init(&argc, &argv);

    // Create pipeline
    GstElement *pipeline = gst_pipeline_new("udp-pipeline");
    GstElement *udpsrc = gst_element_factory_make("udpsrc", "udpsrc");
    GstElement *appsink = gst_element_factory_make("appsink", "appsink");

    if (!pipeline || !udpsrc || !appsink)
    {
        g_printerr("Failed to create GStreamer elements.\n");
        return -1;
    }

    // Set properties for udpsrc
    g_object_set(udpsrc, "port", 5000, nullptr);

    // Set properties for appsink
    g_object_set(appsink, "emit-signals", TRUE, "sync", FALSE, nullptr);
    g_signal_connect(appsink, "new-sample", G_CALLBACK(on_new_sample), nullptr);

    // Add elements to the pipeline
    gst_bin_add_many(GST_BIN(pipeline), udpsrc, appsink, nullptr);
    if (!gst_element_link(udpsrc, appsink))
    {
        g_printerr("Failed to link elements.\n");
        gst_object_unref(pipeline);
        return -1;
    }

    // Set up a message handler to catch errors
    GstBus *bus = gst_element_get_bus(pipeline);
    gst_bus_add_watch(bus, on_message, nullptr);
    gst_object_unref(bus);

    // Start the pipeline
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    // Run the main loop to keep receiving data
    GMainLoop *main_loop = g_main_loop_new(nullptr, FALSE);
    g_main_loop_run(main_loop);

    // Clean up
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    g_main_loop_unref(main_loop);

    return 0;
}
