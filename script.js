const net = require('net');
const flatbuffers = require('flatbuffers');
const turtleSim = require('./turtle_sim').TurtleSim;

const client = new net.Socket();
client.connect(9999, '127.0.0.1', () => {
    console.log('Connected');
});

client.on('data', (data) => {
    const buf = new flatbuffers.ByteBuffer(new Uint8Array(data));
    const turtleStatus = turtleSim.TurtleStatus.getRootAsTurtleStatus(buf);

    console.log('Received: ', turtleStatus.x(), turtleStatus.y(), turtleStatus.theta());
});

client.on('close', () => {
    console.log('Connection closed');
});
