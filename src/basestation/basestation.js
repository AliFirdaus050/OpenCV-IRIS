const WebSocket = require('ws');
const { exec } = require('child_process');

const wss = new WebSocket.Server({ port: 8080 });

wss.on('connection', (ws) => {
    console.log('Basestation connected');

    ws.on('message', (message) => {
        console.log('Received message from ROS: ', message);

        if (message === 'PUBLISH') {
            exec('rosrun ball_detection main_processing_node', (err, stdout, stderr) => {
                if (err) {
                    console.error('Error executing node: ', err);
                    ws.send('Error executing node: ' + err.message);
                    return;
                }
                if (stderr) {
                    console.error('stderr: ', stderr);
                    ws.send('stderr: ' + stderr);
                    return;
                }

                console.log('stdout: ', stdout);
                ws.send('ROS Node Output: ' + stdout);
            });
        }
    });

    ws.send('Basestation ready!');
});

console.log('Basestation server is running on ws://localhost:8080');
