#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <iostream>
#include <vector>
#include <cstring>

GstFlowReturn new_sample(GstAppSink *appsink, gpointer user_data)
{
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample)
    {
        std::cerr << "No sample received from appsink\n";
        return GST_FLOW_ERROR;
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (buffer == nullptr)
    {
        std::cerr << "Buffer is NULL\n";
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_READ))
    {
        std::cout << "Received payload: " << std::string((char *)map.data, map.size) << std::endl;
        gst_buffer_unmap(buffer, &map);
    }
    else
    {
        std::cerr << "Failed to map the buffer\n";
    }

    gst_sample_unref(sample); // Jangan lupa untuk membebaskan memory sample
    return GST_FLOW_OK;
}

int main(int argc, char *argv[])
{
    gst_init(&argc, &argv);

    GstElement *pipeline = gst_pipeline_new("receiver-pipeline");
    GstElement *udpsrc = gst_element_factory_make("udpsrc", "source"); // Deklarasi udpsrc
    GstElement *queue = gst_element_factory_make("queue", "queue");
    GstElement *appsink = gst_element_factory_make("appsink", "sink");

    if (!pipeline || !udpsrc || !queue || !appsink)
    {
        g_printerr("Failed to create elements\n");
        return -1;
    }

    g_object_set(G_OBJECT(udpsrc), "port", 5000, nullptr); // Set port untuk udpsrc

    // Set caps untuk udpsrc
    GstCaps *caps = gst_caps_new_simple("application/x-raw", "media", G_TYPE_STRING, "text", nullptr);
    g_object_set(G_OBJECT(udpsrc), "caps", caps, nullptr);
    gst_caps_unref(caps);

    // Tambahkan elemen ke pipeline
    gst_bin_add_many(GST_BIN(pipeline), udpsrc, queue, appsink, nullptr);

    // Hubungkan elemen
    if (!gst_element_link_many(udpsrc, queue, appsink, nullptr))
    {
        g_printerr("Failed to link udpsrc, queue, and appsink\n");
        return -1;
    }

    // Konfigurasi callback untuk appsink
    g_signal_connect(appsink, "new-sample", G_CALLBACK(new_sample), nullptr);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    g_print("Waiting for payload...\n");

    // Periksa apakah ada message di bus
    GstBus *bus = gst_element_get_bus(pipeline);
    GstMessage *msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE,
                                                 (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    if (msg)
    {
        GError *err;
        gchar *debug_info;
        switch (GST_MESSAGE_TYPE(msg))
        {
        case GST_MESSAGE_ERROR:
            gst_message_parse_error(msg, &err, &debug_info);
            g_printerr("Error: %s\n", err->message);
            g_free(err);
            g_free(debug_info);
            break;
        case GST_MESSAGE_EOS:
            g_print("End of Stream\n");
            break;
        default:
            break;
        }
        gst_message_unref(msg);
    }

    gst_object_unref(bus);

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    return 0;
}
