const net = require('net');
const flatbuffers = require('flatbuffers');
const turtleSim = require('./turtle_sim').TurtleSim;

const PORT = 9898;
const HOST = '127.0.0.1';

const client = new net.Socket();
let isSetupComplete = false;

client.connect(PORT, HOST, () => {
    console.log('Connected to server');

    client.write('SETUP\r\n');
});

client.on('data', (data) => {
    const buf = new flatbuffers.ByteBuffer(new Uint8Array(data));

    if (data.toString().includes("RTSP/1.0 200 OK")) {
        console.log('Received RTSP OK response');

        if (!isSetupComplete) {
            client.write('PLAY\r\n');
            isSetupComplete = true;

            setInterval(() => {
                console.log('Sending PLAY command...');
                client.write('PLAY\r\n');
            }, 1000);
        }
    } else {
        const turtleStatus = turtleSim.TurtleStatus.getRootAsTurtleStatus(buf);

        let x = turtleStatus.x();
        let y = turtleStatus.y();
        let theta = turtleStatus.theta();

        console.log(`Received turtle status:`);
        console.log(`X: ${x}`);
        console.log(`Y: ${y}`);
        console.log(`Theta: ${theta}`);

        // client.write('TEARDOWN\r\n');
    }
});

process.stdin.resume();
process.stdin.on('data', (input) => {
    const command = input.toString().trim();
    if (command == 'quit') {
        console.log('Quitting...');
        client.write('TEARDOWN\r\n');
        client.end();
        process.exit();
    }
});

client.on('close', () => {
    console.log('Connection closed');
});
