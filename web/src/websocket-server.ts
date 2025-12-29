import type { WebSocketServer } from 'ws';
import { SerialPort } from 'serialport';
import { readFileSync } from 'fs';

const escape_map: Record<string,string> = {
    "\\": '\\\\',  // Backslash must be first
    '"': '\\"',    // Double quote
    "'": "\\'",    // Single quote
    '\n': '\\n',   // Newline
    '\r': '\\r',   // Carriage return
    '\t': '\\t',   // Tab
    '\b': '\\b',   // Backspace
    '\f': '\\f',   // Form feed
}
    
function escape_string(val: string) {
    let ret = "";
    for (let v of val) {
        ret = ret + (escape_map[v] ?? v);
    }
    return ret;
}



async function uploadFile(port: SerialPort, srcfile: string, destfile: string) {
    port.write("print(\"Uploading\")\n");

    const lines = readFileSync(srcfile).toString("utf8").split("\n");
    port.write(`tmpf = io.open("${destfile}", "w")\n`);
    for (const l of lines) {
        port.write(`tmpf:write("${escape_string(l.trimEnd())}\\n")\n`);         
    }
    port.write("tmpf:flush()\n");
    port.write("tmpf:close()\n");
    port.write("reboot()\n");
}


export default function setupWebSocket(wss: WebSocketServer) {
    wss.on('connection', socket => {
        try {
            let port: SerialPort | undefined = undefined;

            // port.open(err => {
            //     if (err) {
            //         console.log(err);
            //         socket.send(err.toString() ?? null);
            //     }
            // });

            function connectToSerial() {
                try {
                    port = new SerialPort({
                        path: '/dev/ttyACM0',
                        baudRate: 115200
                    });
                    port.addListener("error", (err) => {
                        console.log("Port error", err);
                    });
                    port.addListener("close", () => {
                        if (socket.readyState == WebSocket.OPEN) {
                            socket.send("Serial Port Closed. Will attempt reopen");
                            setTimeout(connectToSerial, 500);
                        }
                    });

                    port.addListener("data", (chunk: Buffer) => {
                        const sval = chunk.toString("utf8");
                        buf = buf + sval;
                        let idx;
                        while ((idx = buf.indexOf('\n')) != -1) {
                            const line = buf.substring(0, idx);
                            socket.send(line);
                            buf = buf.substring(idx + 1);
                        }
                    });
                    port.write("devices\n");
                } catch (ex) {
                    socket.send(`Error connecting to serial port: ${ex}`);
                    setTimeout(connectToSerial, 500);
                }
            }

            connectToSerial();

            socket.addEventListener("close", () => {
                if (port?.isOpen) {
                    port.close();
                }
            });

            let buf = "";
            socket.addEventListener("message", (evt) => {
                console.log("received", evt.data);
                if (!port) {
                    socket.send("Can not send line: Port is not open");
                } else {
                    if (evt.data == "upload") {
                        uploadFile(port, "/home/bruce/switch_gateway_firmware/src/main.lua", "/lua/main.lua");
                    } else if (evt.data == "upload_bindings") {
                        uploadFile(port, "/home/bruce/TinyModbus/web/bindings.txt", "/bindings.txt");
                    } else {
                        port.write(evt.data + "\n");
                    }
                }
            })
        } catch (ex) {
            console.log(ex);
            socket.close();
        }
    });
}