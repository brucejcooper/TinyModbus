import { Interface } from "readline";
import { Serial, SerialPort } from "./serial";

const crc_table: number[] = [
    0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241, 0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440, 0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
    0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841, 0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40, 0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
    0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641, 0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040, 0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
    0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441, 0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41, 0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
    0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41, 0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40, 0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
    0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041, 0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240, 0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
    0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41, 0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840, 0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
    0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40, 0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640, 0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
    0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241, 0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440, 0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
    0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841, 0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40, 0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
    0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641, 0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040,
];

const modbus_error_codes: Record<number, string> = {
    0x01: "Illegal Function",
    0x02: "Illegal Data Address",
    0x03: "Illegal Data Value",
    0x04: "Slave Device Failure",
    0x05: "Acknowledge",
    0x06: "Slave Device Busy",
    0x07: "Negative Acknowledge",
    0x08: "Memory Parity Error",
    0x09: "Gateway Path unavailable",
    0x0a: "Gateway Target Device Failed to Respond",
};

function calcCrc(arr: Uint8Array) {
    let crc = 0xffff;
    for (let i = 0; i < arr.length - 2; i++) {
        const xorval = (arr[i] ^ crc) & 0xff;
        crc >>= 8;
        crc ^= crc_table[xorval];
    }
    new DataView(arr.buffer, arr.length - 2).setUint16(0, crc, true);
}

function convertUint8_to_hexStr(uint8: Uint8Array) {
    let res = "";
    for (let i = 0; i < uint8.length; i++) {
        res += uint8[i].toString(16).padStart(2, "0");
    }
    return res;
}

interface Command<X> {
    cmdBytes: Uint8Array;
    resolve: (x: X) => void;
    reject: (err: any) => void;
}

type EventHandler = () => void;
type ModbusConnectionEventType = "open" | "close";

export enum ModbusCommand {
    READ_COILS = 0x01,
    READ_DISCRETE_INPUTS = 0x02,
    READ_HOLDING_REGISTERS = 0x03,
    READ_INPUT_REGISTERS = 0x04,
    WRITE_SINGLE_COIL = 0x05,
    WRITE_SINGLE_REGISTER = 0x06,
    WRITE_MULTIPLE_COILS = 0x15,
    WRITE_MULTIPLE_REGISTERS = 0x16,
    MODBUS_CMD_CUSTOM_EXEC_DALI = 0x44,
    MODBUS_CMD_CUSTOM_START_PROCESS = 0x45,
}

function toU16Array(bytes: Uint8Array): number[] {
    const arr = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const res: number[] = [];
    for (let i = 0; i < bytes.byteLength/2; i+= 2) {
        res.push(arr.getUint16(i, false));
    }
    return res;
}

function toBigInt(bytes: Uint8Array) {
    let v = 0n;
    let i = bytes.byteLength;
    while (i) {
        v = (v << 8n) | BigInt(bytes[--i]);
    }
    return v;
}


export class ModbusError extends Error {
    code: number;

    constructor(msg: string, code: number) {
        super(msg);
        this.code = code;
    }
}

function executeWithTimeout<X>(p: Promise<X>, timeout: number): Promise<X> {
    return Promise.race<X>([p, new Promise((_, reject) => {
        setTimeout(reject, timeout, new Error("timeout"));
    })])
}



class ModbusResponseChunker {
    reader: ReadableStreamDefaultReader<Uint8Array<ArrayBuffer>>;
    resetTimer: any = null;
    responseBytes = new Uint8Array(256);
    readLen = 0;
    resolve?: (v: Uint8Array<ArrayBuffer>) => void;
    reject?: (err: any) => void;

    constructor(reader: ReadableStreamDefaultReader<Uint8Array<ArrayBuffer>>) {
        this.reader = reader;
        this.readPackets();
    }

    readTimeout = () => {
        if (this.reject) {
            this.reject(new Error(`Read timeout after reading ${this.readLen} response bytes`));
            this.readLen = 0;
            this.resolve = undefined;
            this.reject = undefined;
        }
    }

    isValid = () => {
        let cmd = this.responseBytes[1];
        let expectedLen = -1;
        if (this.readLen > 4) {
            if (cmd & 0x80) {
                expectedLen = 5;
            } else {
                switch (cmd) {
                    case ModbusCommand.READ_COILS:
                    case ModbusCommand.READ_DISCRETE_INPUTS:
                    case ModbusCommand.READ_HOLDING_REGISTERS:
                    case ModbusCommand.READ_INPUT_REGISTERS:
                        // Length delimited result.
                        expectedLen = this.responseBytes[2] + 5;
                        break;
                    case ModbusCommand.WRITE_SINGLE_COIL:
                    case ModbusCommand.WRITE_SINGLE_REGISTER:
                        expectedLen = 8;
                        break;

                    case ModbusCommand.MODBUS_CMD_CUSTOM_EXEC_DALI:
                    case ModbusCommand.MODBUS_CMD_CUSTOM_START_PROCESS:
                        expectedLen = 5;
                        break;
                }
            }
        }
        return this.readLen == expectedLen;
    };

    readPackets = async () => {
        while (true) {
            this.readLen = 0;
            while (!this.isValid()) {
                const { value: buf, done } = await this.reader.read();
                // Stop any timeout timer.
                clearTimeout(this.resetTimer);
                if (done) {
                    throw new Error("reader closed");
                } else {
                    this.responseBytes.set(buf, this.readLen);
                    this.readLen += buf.length;

                    // The read timeout between reads is actually closer to 1ms, but there are USB and JS delays, so give it a bit more. 
                    this.resetTimer = setTimeout(this.readTimeout, 100);
                }
            }
            clearTimeout(this.resetTimer);
            const out = this.responseBytes.slice(0, this.readLen);
            this.readLen == 0;

            if (this.resolve) {
                this.resolve(out);
            }
            this.resolve = undefined;
            this.reject = undefined;
        }
    }


    read = (): Promise<Uint8Array<ArrayBuffer>> => {
        if (this.resolve) {
            throw new Error("Only one concurrent read call allowed");
        }
        return new Promise<Uint8Array<ArrayBuffer>>((resolve, reject) => {
            this.resolve = resolve;
            this.reject = reject;
            this.resetTimer = setTimeout(this.readTimeout, 1000);
        });
    }
}


export class ModbusConnection {
    port?: SerialPort;

    command: Command<any> | null = null;
    pendingCommands: Command<any>[] = [];
    eventHandlers: Record<ModbusConnectionEventType, EventHandler[]> = {
        "close": [],
        "open": [],
    };

    awaitingPacket: ((pkt: Uint8Array<ArrayBuffer>) => void) | null = null;
    responseReader?: ModbusResponseChunker;

    private addCommand<X>(cmd: Uint8Array): Promise<X> {
        if (!this.port) {
            throw new Error("Not open");
        }
        return new Promise<X>((resolve, reject) => {
            if (this.pendingCommands.push({ cmdBytes: cmd, resolve, reject, }) == 1) {
                this.processCommands();
            }
        });
    }

    addEventHandler(evt: ModbusConnectionEventType, handler: EventHandler) {
        this.eventHandlers[evt].push(handler);
    }
    removeEventHandler(evt: ModbusConnectionEventType, handler: EventHandler) {
        const idx = this.eventHandlers[evt].findIndex(handler);
        if (idx !== -1) {
            this.eventHandlers[evt].splice(idx, 1);
        }
    }

    private dispatch(evt: ModbusConnectionEventType) {
        for (let h of this.eventHandlers[evt]) {
            h();
        }
    }

    setCoil(device: number, address: number, value: boolean) {
        const arr = new Uint8Array(8);
        const dv = new DataView(arr.buffer);
        dv.setUint8(0, device);
        dv.setUint8(1, ModbusCommand.WRITE_SINGLE_COIL);
        dv.setUint16(2, address);
        dv.setUint16(4, value ? 0xff00 : 0x0000);
        return this.addCommand<null>(arr);
    }

    getCoils(device: number, start_address: number, num_coils: number) {
        const arr = new Uint8Array(8);
        const dv = new DataView(arr.buffer);
        dv.setUint8(0, device);
        dv.setUint8(1, ModbusCommand.READ_COILS);
        dv.setUint16(2, start_address);
        dv.setUint16(4, num_coils);
        return this.addCommand<bigint>(arr);
    }

    getDiscreteInputs(device: number, start_address: number, num_coils: number) {
        const arr = new Uint8Array(8);
        const dv = new DataView(arr.buffer);
        dv.setUint8(0, device);
        dv.setUint8(1, ModbusCommand.READ_DISCRETE_INPUTS);
        dv.setUint16(2, start_address);
        dv.setUint16(4, num_coils);
        return this.addCommand<bigint>(arr);
    }

    setHoldingRegister(device: number, address: number, value: number) {
        const arr = new Uint8Array(8);
        const dv = new DataView(arr.buffer);
        dv.setUint8(0, device);
        dv.setUint8(1, ModbusCommand.WRITE_SINGLE_REGISTER);
        dv.setUint16(2, address);
        dv.setUint16(4, value);
        return this.addCommand<null>(arr);
    }

    async startLongRunningProcess(processNum: number) {
        const arr = new Uint8Array(5);
        const dv = new DataView(arr.buffer);
        dv.setUint8(0, 1);
        dv.setUint8(1, ModbusCommand.MODBUS_CMD_CUSTOM_START_PROCESS);
        dv.setUint8(2, processNum);
        await this.addCommand<number>(arr);
    }

    async executeCustomDaliCommand(device: number, cmd: number, repeat = false): Promise<number> {
        const arr = new Uint8Array(7);
        const dv = new DataView(arr.buffer);
        dv.setUint8(0, device);
        dv.setUint8(1, ModbusCommand.MODBUS_CMD_CUSTOM_EXEC_DALI);
        dv.setUint16(2, cmd);
        dv.setUint8(4, repeat ? 2 : 1);
        const res = await this.addCommand<number>(arr);
        // console.log("executing custom command", cmd.toString(16).padStart(4, "0"), "=>", res == null ? "NAK": res.toString(16).padStart(2, '0'));
        return res;
    }

    getHoldingRegisters(device: number, start_address: number, num_registers: number) {
        const arr = new Uint8Array(8);
        const dv = new DataView(arr.buffer);
        dv.setUint8(0, device);
        dv.setUint8(1, ModbusCommand.READ_HOLDING_REGISTERS);
        dv.setUint16(2, start_address);
        dv.setUint16(4, num_registers);

        return this.addCommand<number[]>(arr);
    }

    open = async (port: SerialPort) => {
        console.log("Opening with", port);
        this.port = port;
        await this.port.open({ baudRate: 9600 });
        // An independent process to receive responses. 
        this.responseReader = new ModbusResponseChunker(this.port.readable!.getReader());
        this.dispatch("open");
    }




    private async processCommands() {
        while (this.pendingCommands.length > 0) {
            const { cmdBytes, resolve, reject } = this.pendingCommands[0];
            const writer = this.port?.writable?.getWriter();
            if (!writer || !this.responseReader) {
                throw new Error("No reader");
            }
            try {
                calcCrc(cmdBytes);
                // console.log("Transmitting", convertUint8_to_hexStr(command, command.length));
                writer.write(cmdBytes);
                const response = await this.responseReader.read();

                // The response will be a valid MODBUS response.
                let cmd = response[1];
                let len;

                if (cmd & 0x80) {
                    const errorCode = response[2];
                    // ox01 as gateway device failed to respond - i.e. a NAK
                    if (errorCode == 0x0a) {
                        resolve(null);
                    } else {
                        reject(new ModbusError(modbus_error_codes[errorCode] ?? `Unknown error ${errorCode}`, errorCode));
                    }
                }
                switch (cmd) {
                    case ModbusCommand.READ_COILS:
                    case ModbusCommand.READ_DISCRETE_INPUTS:
                    case ModbusCommand.READ_HOLDING_REGISTERS:
                    case ModbusCommand.READ_INPUT_REGISTERS:
                        // Length delimited result.
                        len = response[2];
                        const responseBytes = response.slice(3, 3 + len);

                        if (cmd <= ModbusCommand.READ_DISCRETE_INPUTS) {
                            // Bitfield response, represented as a bigint.
                            resolve(toBigInt(response));
                        } else {
                            // U16 array response
                            resolve(toU16Array(responseBytes));
                        }
                        break;

                    case ModbusCommand.WRITE_SINGLE_COIL:
                    case ModbusCommand.WRITE_SINGLE_REGISTER:
                        resolve(null);
                        break;

                    case ModbusCommand.MODBUS_CMD_CUSTOM_EXEC_DALI:
                        resolve(response[2]);
                        break;

                    case ModbusCommand.MODBUS_CMD_CUSTOM_START_PROCESS:
                        resolve(null);
                        break;

                    default:
                        reject(new Error("Unknown command"));
                        break;
                }

                // Mandatory timing gap between packets.
                await new Promise((resolve) => setTimeout(resolve, 2));
            } catch (ex) {
                reject(ex);
                // console.error(ex);
            } finally {
                // Remove the pending command from the queue.
                this.pendingCommands.shift();
                writer.releaseLock();
            }
        }
    }

    async close() {
        console.log("closing serial port");
        // Wait for our reader to finish operation.
        await this.port?.close();
        this.port = undefined;
        this.dispatch("close");
    }

    async forget() {
        await this.close();
        await this.port?.forget();
    }
}
