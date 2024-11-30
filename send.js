const dgram = require('dgram');
const flatbuffers = require('flatbuffers');
const TurtleSim = require('./turtle_sim').TurtleSim;

// Create a UDP socket
const socket = dgram.createSocket('udp4');

// Function to send FlatBuffer data
function sendFlatBufferData(x, y, theta) {
    // Create a FlatBuffer builder
    const builder = new flatbuffers.Builder(1024);

    // Create the TurtleStatus FlatBuffer
    TurtleSim.TurtleStatus.startTurtleStatus(builder);
    TurtleSim.TurtleStatus.addX(builder, x);
    TurtleSim.TurtleStatus.addY(builder, y);
    TurtleSim.TurtleStatus.addTheta(builder, theta);
    const turtleStatusOffset = TurtleSim.TurtleStatus.endTurtleStatus(builder);

    // Finish the FlatBuffer
    builder.finish(turtleStatusOffset);

    // Get the buffer and its length
    const buf = builder.asUint8Array();

    // Send the buffer over UDP
    socket.send(buf, 0, buf.length, 5000, '127.0.0.1', (err) => {
        if (err) {
            console.error(`Error sending UDP message: ${err.message}`);
        } else {
            console.log('Message sent successfully');
        }
    });
}

// Example of sending data
sendFlatBufferData(1.0, 2.0, 3.14159);

// Close the socket after some time
setTimeout(() => {
    socket.close();
}, 1000);
