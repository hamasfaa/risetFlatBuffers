const net = require('net');
const flatbuffers = require('flatbuffers');
const turtleSim = require('./turtle_sim').TurtleSim;

const client = new net.Socket();

function createTurtleStatus(x, y, theta) {
    const builder = new flatbuffers.Builder(1024);

    turtleSim.TurtleStatus.startTurtleStatus(builder);
    turtleSim.TurtleStatus.addX(builder, x);
    turtleSim.TurtleStatus.addY(builder, y);
    turtleSim.TurtleStatus.addTheta(builder, theta);

    const turtleStatus = turtleSim.TurtleStatus.endTurtleStatus(builder);

    builder.finish(turtleStatus);

    return builder.asUint8Array();
}

client.connect(1234, '127.0.0.1', () => {
    console.log('[JS] Connected to the server.');
});

client.on('data', (data) => {
    if (data.length > 0) {
        const buf = new flatbuffers.ByteBuffer(new Uint8Array(data));
        const turtleStatus = turtleSim.TurtleStatus.getRootAsTurtleStatus(buf);

        let x = turtleStatus.x();
        let y = turtleStatus.y();
        let theta = turtleStatus.theta();

        // console.log('Received: ', turtleStatus.x(), turtleStatus.y(), turtleStatus.theta());
        console.log(`[JS] Received from Server: x: ${x}, y: ${y}, theta: ${theta}`);

        let response = createTurtleStatus(x + 1, y + 1, theta + 1);
        client.write(response);
        console.log(`[JS] Sending to Server: x: ${x + 1}, y: ${y + 1}, theta: ${theta + 1}`);

    }
});

client.on('close', () => {
    console.log('[JS] Connection closed.');
});
