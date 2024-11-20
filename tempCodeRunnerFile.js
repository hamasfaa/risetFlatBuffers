const rl = readline.createInterface({
//     input: process.stdin,
//     output: process.stdout
// });

// rl.on('line', (input) => {
//     let linear = 0.0;
//     let angular = 0.0;

//     switch (input) {
//         case 'w':
//             linear = 1.0;
//             break;
//         case 's':
//             linear = -1.0;
//             break;
//         case 'a':
//             angular = 1.0;
//             break;
//         case 'd':
//             angular = -1.0;
//             break;
//     }

//     const builder = new flatbuffers.Builder(1024);
//     const ControlCommand = turtleSim.ControlCommand.createControlCommand(builder, linear, angular);
//     builder.finish(ControlCommand);

//     const buf = builder.asUint8Array();
//     client.write(buf);

//     console.log('Sending: ', linear, angular);
// });