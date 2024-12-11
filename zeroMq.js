const zmq = require('zeromq');
const flatbuffers = require('flatbuffers');
const turtleSim = require('./turtle_sim').TurtleSim;

// async function run() {
//     const sock = new zmq.Subscriber();

//     sock.connect("tcp://localhost:5555");

//     sock.subscribe("status");

//     console.log("JavaScript: Waiting for messages on 'status'...");

//     for await (const [topic, msg] of sock) {
//         if (topic.toString() == 'status') {
//             const buf = new flatbuffers.ByteBuffer(new Uint8Array(msg));
//             const turtleStatus = turtleSim.TurtleStatus.getRootAsTurtleStatus(buf);

//             let x = turtleStatus.x();
//             let y = turtleStatus.y();
//             let theta = turtleStatus.theta();

//             console.log(`[JS] 1 Received from Server: x: ${x}, y: ${y}, theta: ${theta}`);
//         }
//     }
// }

// run();

async function run() {
    const sock = new zmq.Publisher();

    await sock.bind("tcp://*:5556");

    while (true) {
        console.log("sending a multipart message envelope")
        await sock.send(["kitty cats", "meow!"])
        await new Promise(resolve => {
            setTimeout(resolve, 500)
        })
    }
}

run();