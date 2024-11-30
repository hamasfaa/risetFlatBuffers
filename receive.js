const dgram = require('dgram');
const flatbuffers = require('flatbuffers');
const TurtleSim = require('./turtle_sim').TurtleSim;

// Create UDP socket to receive data
const PORT = 5000;
const HOST = '127.0.0.1';
const socket = dgram.createSocket('udp4');

socket.on('message', (msg) => {
    // Create ByteBuffer from received message
    const byteBuffer = new flatbuffers.ByteBuffer(new Uint8Array(msg));

    // Deserialize the data using FlatBuffers
    const turtleStatus = TurtleSim.TurtleStatus.getRootAsTurtleStatus(byteBuffer);

    // Extract values
    const x = turtleStatus.x();
    const y = turtleStatus.y();
    const theta = turtleStatus.theta();

    console.log(`Received Turtle Status: x = ${x}, y = ${y}, theta = ${theta}`);
});

socket.bind(PORT, HOST, () => {
    console.log(`UDP server listening on ${HOST}:${PORT}`);
});