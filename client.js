// const net = require('net');
// const readline = require('readline');

// const PORT = 554;  // RTSP default port
// const HOST = '127.0.0.1';

// // Create TCP connection for RTSP
// const client = new net.Socket();

// client.connect(PORT, HOST, () => {
//     console.log('[Client] Connected to RTSP server');

//     // Send DESCRIBE request to RTSP server
//     const describeRequest = "DESCRIBE rtsp://127.0.0.1/ RTSP/1.0\r\nCSeq: 1\r\n";
//     client.write(describeRequest);
// });

// // Handle incoming RTSP response
// client.on('data', (data) => {
//     const response = data.toString();

//     console.log(`[Client] Received response:\n${response}`);

//     if (response.includes('RTSP/1.0 200 OK')) {
//         // Extract the body containing the integer data (12345)
//         const body = response.split("\r\n\r\n")[1];
//         console.log(`[Client] Received from Server: Data = ${body}`);

//         // Send back the received data to the server (54321)
//         const message = `SETUP rtsp://127.0.0.1/ RTSP/1.0\r\nCSeq: 2\r\nMessage: 54321\r\n`;
//         client.write(message);
//     } else if (response.includes('Acknowledged')) {
//         // Server acknowledges the data sent back
//         console.log('[Client] Server acknowledged the data.');
//         client.end();
//     } else {
//         console.error('[Client] RTSP Error: Invalid response');
//     }
// });

// client.on('error', (err) => {
//     console.error('[Client] Error: ' + err.message);
//     client.end();
// });
const net = require('net');
const readline = require('readline');

const PORT = 554;  // RTSP default port
const HOST = '127.0.0.1';

// Create TCP connection for RTSP
const client = new net.Socket();

client.connect(PORT, HOST, () => {
    console.log('[Client] Connected to RTSP server');

    // Send data to server (e.g., '54321')
    const message = "54321";  // Data to send to the server
    const rtspRequest = `SETUP rtsp://127.0.0.1/ RTSP/1.0\r\nCSeq: 1\r\nMessage: ${message}\r\n`;

    client.write(rtspRequest);
    console.log(`[Client] Sent message: ${message}`);
});

// Handle incoming RTSP response (server acknowledgment)
client.on('data', (data) => {
    const response = data.toString();
    console.log(`[Client] Received response:\n${response}`);

    // If server acknowledges, close connection
    if (response.includes('Acknowledged')) {
        console.log('[Client] Server acknowledged the message.');
        client.end();
    } else {
        console.error('[Client] RTSP Error: Invalid response');
    }
});

client.on('error', (err) => {
    console.error('[Client] Error: ' + err.message);
    client.end();
});
