// const gst = require('child_process');
// const flatbuffers = require('flatbuffers');
// const TurtleSim = require('./turtle_sim').TurtleSim;

// const pipeline = gst.spawn('gst-launch-1.0', [
//     'rtspsrc', 'location=rtsp://127.0.0.1:8080/stream', '!',
//     'rtph264depay', '!', 'appsink', 'name=appsink'
// ]);

// pipeline.stderr.on('data', (data) => {
//     console.error(`Error: ${data}`);
// });

// pipeline.stdout.on('data', (data) => {
//     // Convert the received buffer into FlatBuffer format
//     const byteBuffer = new flatbuffers.ByteBuffer(new Uint8Array(data));

//     // Deserialize the data using FlatBuffers
//     const turtleStatus = TurtleSim.TurtleStatus.getRootAsTurtleStatus(byteBuffer);

//     // Extract values
//     const x = turtleStatus.x();
//     const y = turtleStatus.y();
//     const theta = turtleStatus.theta();

//     console.log(`Received Turtle Status: x = ${x}, y = ${y}, theta = ${theta}`);
// });

// pipeline.on('exit', (code) => {
//     console.log(`Pipeline exited with code ${code}`);
// });
const { spawn } = require('child_process');

// Fungsi untuk menerima data RTSP dari server
function receiveRTSP() {
    // Perintah GStreamer untuk menerima aliran RTSP
    const gstProcess = spawn('gst-launch-1.0', [
        'rtspsrc', 'location=rtsp://localhost:8554/test', '!', 'decodebin', '!', 'fakesink'
    ]);

    // Menangani output standar (stdout)
    gstProcess.stdout.on('data', (data) => {
        console.log(`GStreamer Output: ${data}`);
    });

    // Menangani error standar (stderr)
    gstProcess.stderr.on('data', (data) => {
        console.error(`GStreamer Error: ${data}`);
    });

    // Menangani proses selesai
    gstProcess.on('close', (code) => {
        console.log(`GStreamer process exited with code ${code}`);
    });
}

// Mulai penerima RTSP
receiveRTSP();
