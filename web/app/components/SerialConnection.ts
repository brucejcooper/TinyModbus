import { GatewayDeviceState } from "../deviceState";
import { SerialPort } from "./serial";


type DeviceChangedListener = (state: GatewayDeviceState) => void;
type ConsoleListener = (msg: string) => void;


function isAlphanumeric(char: string) {
    return /^[a-z0-9]$/i.test(char);
}



class Tokenizer {
    str: string;
    pos: number;

    constructor(str: string) {
        this.str = str;
        this.pos = 0;
    }

    expect(char: string) {
        if (this.str[this.pos] !== char) {
            throw new Error(`Expected ${char} at position ${this.pos}, but got ${this.str[this.pos]}`);
        }
        this.pos++;
    }

    identifier(): string {
        const start = this.pos;
        while (isAlphanumeric(this.str[this.pos])) {
            this.pos++;
        }
        return this.str.substring(start, this.pos);
    }

    isNumber() {
        const cc = this.str.charCodeAt(this.pos) - 48;
        return cc >= 0 && cc <= 9
    }


    numericIndex(): number {
        this.expect('[');
        const start = this.pos;
        if (!this.isNumber()) {
            throw new Error(`Expected at least one digit, but got ${this.str[this.pos]}`);
        }
        this.pos++;
        while (this.isNumber()) {
            this.pos++;
        }
        const strVal = this.str.substring(start, this.pos);
        this.expect(']');
        return parseInt(strVal);
    }

    nextPathElement(): string|number|null {
        if (this.pos > this.str.length) {
            throw new Error("Overflow")
        }
        const startChar = this.str[this.pos];
        if (isAlphanumeric(startChar)) {
            return this.identifier()
        } 
        if (startChar == '[') {
            return this.numericIndex();
        }
        if (startChar == ".") {
            this.pos++;
            return this.identifier();
        }
        if (startChar == '=') {
            this.pos++;
            return null;
        }
        throw new Error(`Parse Error at position ${this.pos} - Unexpected character ${startChar}`)
    }

    tokenize(): [(string|number)[], any] {
        let tok: string|number|null;

        const path: (string|number)[] = []
        while ((tok = this.nextPathElement()) !== null) {
            path.push(tok);                    
        }
        const remainder = this.str.substring(this.pos);
        return [path, JSON.parse(remainder)]
    }
}

export class SerialConnection {
    port?: SerialPort;
    closed = true;

    listeners: DeviceChangedListener[] = [];
    console_listeners: ConsoleListener[]= [];

    state: GatewayDeviceState|null = null;

    open = async (port: SerialPort) => {
        console.log("Opening serial port", port);
        this.port = port;
        await this.port.open({ baudRate: 460800 });
        console.log("Port now open.  Starting read loop");
        // An independent process to receive responses. 
        await this.requestDevices();
        console.log("Starting read loop");
        this.readLoop();



        // setTimeout(this.dumpDevices, 8000);
        // this.dispatch("open");
    }


    processIncomingLine = (line: string) => {
        try {            
            if (line.startsWith("{")) {
                // Its a full statement
                this.state = JSON.parse(line);
                this.dispatchChange();
                return;
            } 

            try {
                const [path, val] = new Tokenizer(line).tokenize();
                console.log("Path", path, "=", val);

                const newState: GatewayDeviceState = {...this.state!};
                let parentObj: any = newState;
                const lastPe = path.pop()!;
                for (let pe of path) {
                    const existing = parentObj[pe];
                    const copy = Array.isArray(existing) ? [...existing] : {...existing};
                    parentObj[pe] = copy;
                    parentObj = copy;
                }
                parentObj[lastPe] = val;
                this.state = newState;
                this.dispatchChange();


            } catch (ex) {
                // That is okay.. It just means it isn't a line.
            }
        } catch (ex) {
            console.log("Error processing line", line, ex)
        }
    }

    sendLine = async (line: string) => {
        const encoder = new TextEncoder();
        const writer = this.port!.writable!.getWriter()
        const buf = new Uint8Array(1000);
        buf[0] = 1; // This will disable echo for this command
        const res = encoder.encodeInto(line + "\n", buf.subarray(1));

        const toSend = buf.subarray(0,res.written + 1);

        await writer.write(toSend);
        writer.releaseLock();
    }

    resetDevice = async () => {
        await this.sendLine("")
        await this.sendLine("reboot()")
    }

    requestDevices = async () => {
        console.log("Requesting Devices")
        await this.sendLine("")
        await this.sendLine("devices")
    }

    addChangeListener(listener: DeviceChangedListener) {
        this.listeners.push(listener);
    }

    addConsoleListener(listener: ConsoleListener) {
        this.console_listeners.push(listener);
    }


    removeConsoleListener(listener: ConsoleListener) {
        const idx = this.console_listeners.indexOf(listener);
        if (idx !== -1) {
            this.console_listeners.splice(idx, 1);
        }
    }

    removeChangeListener(listener: DeviceChangedListener) {
        const idx = this.listeners.indexOf(listener);
        if (idx !== -1) {
            this.listeners.splice(idx, 1);
        }
    }

    dispatchChange() {
        if (this.state) {
            for (const l of this.listeners) {
                l(this.state);
            }
        }
    }

    dispatchConsole(line: string) {
        for (const l of this.console_listeners) {
            l(line);
        }
    }



    readLoop = async () => {
        const reader = this.port!.readable!.getReader()
        this.closed = false;
        let buf = "";
        const decoder = new TextDecoder();

        while (!this.closed) {
            const { done, value } = await reader.read()
            if (done) {
                closed = true;
            }
            if (value) {
                buf = buf + decoder.decode(value)
                let nlPos;
                while ((nlPos = buf.indexOf("\n")) != -1) {
                    const line = buf.substring(0, nlPos).trimEnd();
                    this.dispatchConsole(line);
                    this.processIncomingLine(line);
                    buf = buf.substring(nlPos + 1);
                }
            }
        }
        console.log("Serial Port closed");
    }

    async close() {
        console.log("closing serial port");
        // Wait for our reader to finish operation.
        await this.port?.close();
        this.port = undefined;
        // this.dispatch("close");
    }

    async forget() {
        await this.close();
        await this.port?.forget();
    }
}
