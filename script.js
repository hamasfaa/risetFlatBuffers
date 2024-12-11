const zmq = require('zeromq');
const flatbuffers = require('flatbuffers');
const turtleSim = require('./turtle_sim').TurtleSim;

async function run() {
    const subSock = new zmq.Subscriber();
    const pubSock = new zmq.Publisher();

    subSock.connect("tcp://localhost:5555");
    pubSock.bind("tcp://*:5556");

    subSock.subscribe("status");

    console.log("JavaScript: Waiting for messages on 'status'...");

    for await (const [topic, msg] of subSock) {
        if (topic.toString() == 'status') {
            const buf = new flatbuffers.ByteBuffer(new Uint8Array(msg));
            const turtleStatus = turtleSim.TurtleStatus.getRootAsTurtleStatus(buf);

            let x = turtleStatus.x();
            let y = turtleStatus.y();
            let theta = turtleStatus.theta();

            console.log(`[JS] Received from Server: x: ${x}, y: ${y}, theta: ${theta}`);
        }
    }
}

run();