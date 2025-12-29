import { GatewayDeviceState } from "./deviceState";


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

    nextPathElement(): string | number | null {
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

    tokenize(): [(string | number)[], any] {
        let tok: string | number | null;

        const path: (string | number)[] = []
        while ((tok = this.nextPathElement()) !== null) {
            path.push(tok);
        }
        const remainder = this.str.substring(this.pos);
        return [path, JSON.parse(remainder)]
    }
}

export class SerialConnection {
    listeners: DeviceChangedListener[] = [];
    console_listeners: ConsoleListener[] = [];
    state: GatewayDeviceState | null = null;
    ws?: WebSocket;

    open = async () => {
        this.ws = new WebSocket("/ws");
        this.ws.addEventListener("message", this.msgReceived);
        this.ws.addEventListener("close", this.wsClosed);
    }

    msgReceived = (evt: MessageEvent<any>) => {
        const line = evt.data;
        this.dispatchConsole(line);
        this.processIncomingLine(line);

    }

    wsClosed = (evt: CloseEvent) => {
        console.log("Websocket closed");
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
                // console.log("Path", path, "=", val);

                const newState: GatewayDeviceState = { ...this.state! };
                let parentObj: any = newState;
                const lastPe = path.pop()!;
                for (let pe of path) {
                    const existing = parentObj[pe];
                    const copy = Array.isArray(existing) ? [...existing] : { ...existing };
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
        if (!this.ws) {
            throw new Error("WS not open");
        }
        this.ws.send(line);
    }

    resetDevice = async () => {
        await this.sendLine("");
        await this.sendLine("reboot()");
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

    async close() {
        console.log("closing serial port");
        // Wait for our reader to finish operation.
        // this.dispatch("close");
    }

    async forget() {
        await this.close();
    }
}
