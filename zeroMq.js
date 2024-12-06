const zmq = require('zeromq');
const flatbuffers = require('flatbuffers');
const turtleSim = require('./turtle_sim').TurtleSim;

async function run() {
    const sock = new zmq.Pull();

    sock.connect("tcp://localhost:5555");
    console.log("JavaScript: Waiting for message...");

    for await (const [msg] of sock) {
        const buf = new flatbuffers.ByteBuffer(new Uint8Array(msg));
        const turtleStatus = turtleSim.TurtleStatus.getRootAsTurtleStatus(buf);

        let x = turtleStatus.x();
        let y = turtleStatus.y();
        let theta = turtleStatus.theta();

        console.log(`[JS] Received from Server: x: ${x}, y: ${y}, theta: ${theta}`);
    }
}

run();