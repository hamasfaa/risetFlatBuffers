const dgram = require('dgram');
const flatbuffers = require('flatbuffers');
const turtleSim = require('./turtle_sim').TurtleSim;
const readline = require('readline');

const PORT = 1234;
const HOST = '127.0.0.1';
const client = dgram.createSocket('udp4');

function createTurtleStatus(linear, angular) {
    const builder = new flatbuffers.Builder(1024);

    turtleSim.ControlCommand.startControlCommand(builder);
    turtleSim.ControlCommand.addLinearVelocity(builder, linear);
    turtleSim.ControlCommand.addAngularVelocity(builder, angular);

    const controlCommand = turtleSim.ControlCommand.endControlCommand(builder);

    builder.finish(controlCommand);

    return builder.asUint8Array();
}

client.on('message', (msg, rinfo) => {
    const buf = new flatbuffers.ByteBuffer(new Uint8Array(msg));
    const turtleStatus = turtleSim.TurtleStatus.getRootAsTurtleStatus(buf);

    let x = turtleStatus.x();
    let y = turtleStatus.y();
    let theta = turtleStatus.theta();

    console.log(`[JS] Received from Server: x: ${x}, y: ${y}, theta: ${theta}`);
});

client.on('error', (err) => {
    console.error(`[JS] Error: ${err.message}`);
    client.close();
});

const rl = readline.createInterface({
    input: process.stdin,
    output: process.stdout
});

rl.on('line', (input) => {
    let linearVelocity = 0;
    let angularVelocity = 0;

    switch (input) {
        case 'w':
            linearVelocity = 1;
            break;
        case 's':
            linearVelocity = -1;
            break;
        case 'a':
            angularVelocity = 1;
            break;
        case 'd':
            angularVelocity = -1;
            break;
        default:
            console.log('Invalid input. Use w/s for linear and a/d for angular velocity.');
            return;
    }

    const response = createTurtleStatus(linearVelocity, angularVelocity);
    client.send(response, 0, response.length, PORT, HOST)
});
