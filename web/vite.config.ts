import { defineConfig } from "vite";
import ws from 'vite-plugin-ws';
import setupWebSocket from "./src/websocket";


export default defineConfig({
    plugins: [
        ws({
            path: '/ws',
            setup: setupWebSocket
        })
    ]
});
