// const fs = require('fs');
const net = require('net');
const flatbuffers = require('flatbuffers');
const myGame = require('./monster').myGame;


function createMonster() {
    const builder = new flatbuffers.Builder(1024);

    const name = builder.createString('Eren Yeager');
    myGame.Monster.startMonster(builder);
    myGame.Monster.addId(builder, 10);
    myGame.Monster.addName(builder, name);
    myGame.Monster.addHp(builder, 100);
    const monster = myGame.Monster.endMonster(builder);

    builder.finish(monster);

    return builder.asUint8Array();
}

const client = new net.Socket();
client.connect(9999, '127.0.0.1', () => {
    console.log('Connected');
});

client.on('data', (data) => {
    const byteBuffer = new flatbuffers.ByteBuffer(new Uint8Array(data));
    const monster = myGame.Monster.getRootAsMonster(byteBuffer);

    console.log("FROM C++");
    console.log("Monster ID:", monster.id());
    console.log("Monster Name:", monster.name());
    console.log("Monster HP:", monster.hp());

    client.write(createMonster());
});

client.on('close', () => {
    console.log('Connection closed');
});
