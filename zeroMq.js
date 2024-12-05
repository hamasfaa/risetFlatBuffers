const zmq = require('zeromq');

async function run() {
    const sock = new zmq.Pull();

    sock.connect("tcp://localhost:5555");
    console.log("JavaScript: Waiting for message...");

    for await (const [msg] of sock) {
        console.log("JavaScript: Received message:", msg.toString());
    }
}

run();