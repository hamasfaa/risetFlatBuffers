#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/rtsp-server/rtsp-media-factory.h>
#include <gst/gstbuffer.h>

// Fungsi untuk memproses buffer dan mencetak metadata
static GstPadProbeReturn probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    if (buffer)
    {
        guint8 *data = GST_BUFFER_INFO_DATA(buffer);
        guint size = GST_BUFFER_INFO_SIZE(buffer);

        // Anggap metadata adalah string yang disisipkan pada buffer (seperti "hello world")
        gchar *metadata = g_strndup((char *)data, size);
        g_print("[Client] Received metadata: %s\n", metadata);
        g_free(metadata);
    }

    return GST_PAD_PROBE_OK;
}

int main(int argc, char *argv[])
{
    gst_init(&argc, &argv);

    GstElement *pipeline = gst_parse_launch("rtspsrc location=rtsp://127.0.0.1:8554/test ! decodebin ! autovideosink", NULL);

    GstPad *pad = gst_element_get_static_pad(pipeline, "src");

    // Menambahkan probe untuk memeriksa buffer video
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, probe_callback, NULL, NULL);

    GstBus *bus = gst_element_get_bus(pipeline);
    GstMessage *msg;

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    while (true)
    {
        msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE, GST_MESSAGE_ERROR | GST_MESSAGE_EOS);
        if (msg != NULL)
        {
            GError *err;
            gchar *debug_info;
            switch (GST_MESSAGE_TYPE(msg))
            {
            case GST_MESSAGE_ERROR:
                gst_message_parse_error(msg, &err, &debug_info);
                g_print("Error: %s\n", err->message);
                g_error_free(err);
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
    }

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(bus);
    gst_object_unref(pipeline);

    return 0;
}
