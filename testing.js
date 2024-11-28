const net = require('net');
const flatbuffers = require('flatbuffers');
const turtleSim = require('./turtle_sim').TurtleSim;

const PORT = 12345;
const HOST = '127.0.0.1';

// Connect to the C++ server
const client = new net.Socket();
client.connect(PORT, HOST, () => {
    console.log('Connected to server');

    // Send SETUP command
    client.write('SETUP\r\n');
});

client.on('data', (data) => {
    // Convert the incoming data into a byte array
    const byteArray = new Uint8Array(data);
    const buf = new flatbuffers.ByteBuffer(byteArray);

    // Check for RTSP response before sending PLAY
    if (data.toString().includes("RTSP/1.0 200 OK")) {
        console.log('Received RTSP OK response');

        // Wait for RTSP response to complete before sending PLAY
        client.write('PLAY\r\n'); // Add a small delay to ensure proper sequencing
    } else {
        // Deserialize the FlatBuffer data
        const turtleStatus = turtleSim.TurtleStatus.getRootAsTurtleStatus(buf);

        console.log(`Received turtle status:`);
        console.log(`X: ${turtleStatus.x()}`);
        console.log(`Y: ${turtleStatus.y()}`);
        console.log(`Theta: ${turtleStatus.theta()}`);

        // Send TEARDOWN command after receiving the data
        client.write('TEARDOWN\r\n');

    }
});

client.on('close', () => {
    console.log('Connection closed');
});