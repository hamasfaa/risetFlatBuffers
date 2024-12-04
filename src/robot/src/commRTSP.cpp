#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <cstring>
#include <vector>

GstBuffer *text_to_buffer(const char *text)
{
    int size = strlen(text);
    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    memcpy(map.data, text, size);
    gst_buffer_unmap(buffer, &map);
    return buffer;
}

void push_data(GstElement *appsrc, const char *text)
{
    GstBuffer *buffer = text_to_buffer(text);
    GstFlowReturn ret;
    g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    if (ret != GST_FLOW_OK)
    {
        g_printerr("Failed to push buffer to appsrc\n");
    }
    else
    {
        g_print("Payload pushed\n");
    }
}

int main(int argc, char *argv[])
{
    gst_init(&argc, &argv);

    GstElement *pipeline = gst_pipeline_new("transmitter-pipeline");
    GstElement *appsrc = gst_element_factory_make("appsrc", "source");
    GstElement *queue = gst_element_factory_make("queue", "queue");
    GstElement *udpsink = gst_element_factory_make("udpsink", "sink");

    if (!pipeline || !appsrc || !queue || !udpsink)
    {
        g_printerr("Failed to create elements\n");
        return -1;
    }

    // Konfigurasi elemen
    g_object_set(G_OBJECT(udpsink), "host", "127.0.0.1", "port", 5000, nullptr);

    // Tambahkan elemen ke pipeline
    gst_bin_add_many(GST_BIN(pipeline), appsrc, queue, udpsink, nullptr);

    // Hubungkan elemen
    if (!gst_element_link_many(appsrc, queue, udpsink, nullptr))
    {
        g_printerr("Failed to link appsrc, queue, and udpsink\n");
        return -1;
    }

    // Konfigurasi appsrc
    GstCaps *caps = gst_caps_new_simple(
        "application/x-raw",
        "media", G_TYPE_STRING, "text",
        nullptr);
    g_object_set(G_OBJECT(appsrc), "caps", caps, nullptr);
    gst_caps_unref(caps);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    const char *text = "Yuke ganteng banget anjiay";
    push_data(appsrc, text);

    g_usleep(5000000); // Tunggu 5 detik
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    return 0;
}