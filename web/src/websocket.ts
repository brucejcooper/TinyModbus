import type { WebSocketServer } from 'ws';

export default function setupWebSocket(wss: WebSocketServer) {
    wss.on('connection', socket => {
        socket.send('Connected!');
    });
}