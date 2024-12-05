const dgram = require('dgram');
const gst = require('gstreamer-superficial');
const flatbuffers = require('flatbuffers');
const TurtleSim = require('./turtle_sim').TurtleSim;

function createAndSendData(appsrc) {
    // Create FlatBuffer data
    const builder = new flatbuffers.Builder(1024);
    const ts = TurtleSim.TurtleStatus.createTurtleStatus(builder, 1.0, 2.0, 3.14159);
    builder.finish(ts);

    const buffer = Buffer.from(builder.asUint8Array());

    // Create GStreamer buffer and push to appsrc
    const gstBuffer = gst.Buffer.wrap(buffer);
    appsrc.push(gstBuffer);
}

function receiveAndProcessData(appsink) {
    const sample = appsink.pullSample();
    if (!sample) {
        console.error('Failed to pull sample from appsink.');
        return;
    }

    const buffer = sample.buffer;
    const data = buffer.data;

    // Verify and parse FlatBuffer data
    const bytes = new Uint8Array(data);
    if (!TurtleStatus.bufferHasIdentifier(bytes)) {
        console.error('Received invalid FlatBuffer data.');
    } else {
        const turtleStatus = TurtleStatus.getRootAsTurtleStatus(new flatbuffers.ByteBuffer(bytes));
        const x = turtleStatus.x();
        const y = turtleStatus.y();
        const theta = turtleStatus.theta();

        console.log(`Received Turtle Status: x = ${x}, y = ${y}, theta = ${theta}`);
    }
}

function main() {
    // Create the GStreamer pipeline for sending data
    const pipeline = new gst.Pipeline('appsrc name=source ! appsink name=sink');

    // Mendapatkan elemen appsrc dan appsink dari pipeline
    const appsrc = pipeline.findChild('source');
    const appsink = pipeline.findChild('sink');

    if (!pipeline || !appsrc || !appsink) {
        console.error('Failed to create GStreamer elements.');
        return;
    }

    // Send data using appsrc
    createAndSendData(appsrc);

    // Receive and process data from appsink
    receiveAndProcessData(appsink);

    pipeline.setState(gst.State.NULL);
}

main();