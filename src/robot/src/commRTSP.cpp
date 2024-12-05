#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/rtsp-server/rtsp-media-factory.h>
#include <stdio.h>

static GstElement *pipeline;
static GstElement *video_source, *x264enc, *rtph264pay, *udp_sink;
static GstRTSPServer *server;
static GstRTSPMediaFactory *factory;

static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data)
{
    switch (GST_MESSAGE_TYPE(msg))
    {
    case GST_MESSAGE_ERROR:
    {
        gchar *debug;
        GError *err;
        gst_message_parse_error(msg, &err, &debug);
        g_free(debug);
        g_error_free(err);
        break;
    }
    default:
        break;
    }
    return TRUE;
}

static GstPadProbeReturn probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data)
{
    // Sisipkan metadata dalam paket video
    static guint8 metadata[256];
    static int metadata_len = 0;
    if (metadata_len == 0)
    {
        // Menyisipkan "hello world" dalam metadata
        metadata_len = snprintf((char *)metadata, sizeof(metadata), "hello world");
    }

    // Mengirim metadata ke dalam stream, misalnya ke buffer
    GstBuffer *buffer = gst_buffer_new_and_alloc(metadata_len);
    gst_buffer_fill(buffer, 0, metadata, metadata_len);
    gst_pad_push(pad, buffer);

    return GST_PAD_PROBE_OK;
}

int main(int argc, char *argv[])
{
    gst_init(&argc, &argv);

    // Membuat RTSP server
    server = gst_rtsp_server_new();
    gst_rtsp_server_attach(server, NULL);

    // Membuat pipeline GStreamer
    video_source = gst_element_factory_make("videotestsrc", "video_source");
    x264enc = gst_element_factory_make("x264enc", "x264enc");
    rtph264pay = gst_element_factory_make("rtph264pay", "rtph264pay");
    udp_sink = gst_element_factory_make("udpsink", "udp_sink");

    pipeline = gst_pipeline_new("video_pipeline");
    gst_bin_add_many(GST_BIN(pipeline), video_source, x264enc, rtph264pay, udp_sink, NULL);
    gst_element_link_many(video_source, x264enc, rtph264pay, udp_sink, NULL);

    // Mengatur properti pipeline
    g_object_set(udp_sink, "host", "127.0.0.1", NULL);
    g_object_set(udp_sink, "port", 5000, NULL);

    // Menambahkan probe untuk menyisipkan metadata
    GstPad *pad = gst_element_get_static_pad(rtph264pay, "src");
    gst_pad_add_probe(pad, GST_PAD_PROBE_TYPE_BUFFER, probe_callback, NULL, NULL);

    // Menjalankan pipeline
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    // Membuat factory untuk stream RTSP
    factory = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(factory, "( videotestsrc ! x264enc ! rtph264pay pt=96 name=pay0 )");

    // Menambahkan factory ke server RTSP
    GstRTSPMountPoints *mounts = gst_rtsp_server_get_mount_points(server);
    gst_rtsp_mount_points_add_factory(mounts, "/test", factory);

    // Menunggu koneksi
    g_print("RTSP server is running at rtsp://127.0.0.1:8554/test\n");
    gst_rtsp_server_attach(server, NULL);

    // Menunggu input dan menutup server jika diperlukan
    g_main_loop_run(g_main_loop_new(NULL, FALSE));

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    gst_object_unref(server);

    return 0;
}
