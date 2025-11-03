import { SerialPort } from "./serial";

export interface ModbusDeviceValues {
    coils: number[];
    holding: number[];
    inputs: number[];
    failures: number;
}

const topicRe = /^([a-z0-9_\-\/]+):\s?/


type EntityChangedListener = (changed: Entity) => void;
type DeviceChangedListener = (changed: HADevice) => void;


interface DeviceEntityMsg {
    "~": string;
    name?: string;
    stat_t?: string;
    cmd_t?: string;
    availability_topic?: string,
    unique_id: string;
    p: string;
}

interface HADeviceMsg {
    dev: {
        ids: string[],
        name: string,
        via_device?: string,
    },
    cmps: DeviceEntityMsg[]
}

const tenc = new TextEncoder();

export class Entity {
    id: string;
    platform: string;
    value: any = undefined;
    device: HADevice;
    name?: string;

    static allEntities = new Map<string, Entity>();

    constructor(dev: HADevice, msg: DeviceEntityMsg) {
        this.device = dev;
        this.id = msg.unique_id;
        this.platform = msg.p;
        this.name = msg.name;
        Entity.allEntities.set(this.id, this);
    }


    is_on(): boolean {
        switch (this.platform) {
            case 'light':
                return this.value?.state == "ON";
            case 'binary_sensor':
                return this.value?.toUpperCase() == "ON";
            default:
                return false;
        }
    }

    setState(state:any) {
        const current = this.value
        if (current && typeof current == 'object' && typeof(state) === 'object') {
            console.log("assigning", state, "to", this.value);
            Object.assign(this.value, state);
            console.log("Value now", this.value);
        } else {
            this.value = state;
        }
        this.device.dispatchEntityChangeEvent(this)
    }
}



export class HADevice {
    id: string;
    name: string;
    entities: Entity[] = [];
    parent_device: HADevice | undefined = undefined;

    child_devices: HADevice[] = [];

    static allDevices = new Map<string, HADevice>();

    listeners: DeviceChangedListener[] = []
    entityChangelisteners: EntityChangedListener[] = []

    constructor(msg: HADeviceMsg) {
        this.id = msg.dev.ids[0];
        this.name = msg.dev.name;

        HADevice.allDevices.set(this.id, this);
        if (msg.dev.via_device) {
            this.parent_device = HADevice.allDevices.get(msg.dev.via_device);
            this.parent_device!.addChildDevice(this);
        }

        for (const emsg of msg.cmps) {
            this.addEntity(emsg)
        }
    }

    addChangeListener(l: DeviceChangedListener) {
        this.listeners.push(l);

    }
    removeChangeListener(l: DeviceChangedListener) {
        const idx = this.listeners.indexOf(l);
        if (idx !== -1) {
            this.listeners.splice(idx, 1);
        }
    }

    addEntityChangeListener(l: EntityChangedListener) {
        this.entityChangelisteners.push(l);

    }
    removeEntityChangeListener(l: EntityChangedListener) {
        const idx = this.entityChangelisteners.indexOf(l);
        if (idx !== -1) {
            this.entityChangelisteners.splice(idx, 1);
        }
    }

    addChildDevice(dev: HADevice) {
        this.child_devices.push(dev);
        dev.parent_device = this;
        dev.dispatchChangeEvent();
        this.dispatchChangeEvent();

    }

    addEntity(msg: DeviceEntityMsg) {
        const e = new Entity(this, msg)
        this.entities.push(e);
        this.dispatchEntityChangeEvent(e);
        this.dispatchChangeEvent();
    }

    dispatchChangeEvent() {
        this.listeners.forEach(l => l(this));
    }

    dispatchEntityChangeEvent(ent: Entity) {
        this.entityChangelisteners.forEach(l => l(ent));
    }
}





export class SerialConnection extends HADevice {
    port?: SerialPort;
    closed = true;

    changeList = new Set<string>();
    notificationTimer: number = -1;


    constructor() {
        super({ dev: { ids: ["root"], name:"Root"}, cmps: []});
    }


    open = async (port: SerialPort) => {
        console.log("Opening serial port", port);
        this.port = port;
        await this.port.open({ baudRate: 115200 });
        console.log("Port now open.  Starting read loop");
        // An independent process to receive responses. 
        await this.resetDevice();
        console.log("Starting read loop");
        this.readLoop();

        // setTimeout(this.dumpDevices, 8000);
        // this.dispatch("open");
    }

    findOrUpdate(msg: HADeviceMsg): HADevice {
        let dev = HADevice.allDevices.get(msg.dev.ids[0]);
        if (!dev) {
            dev = new HADevice(msg);
            if (!dev.parent_device) {
                this.addChildDevice(dev);
            }
        } else {
            // TODO update device with new config.
            dev.dispatchChangeEvent();
        }
        return dev;
    }


    processIncomingLine = (line: string) => {
        try {
            if (line.match(topicRe)) {

                const colPos = line.indexOf(':');
                const [platform, entity_id, subtopic] = line.substring(0, colPos).split('/');
                let msg: string | object = line.substring(colPos + 1);
                if (msg.startsWith("{")) {
                    msg = JSON.parse(msg);
                }

                console.log("<", line);

                switch (subtopic) {
                    case "available":
                        // console.log("Entity", entity_id, msg)
                        break;
                    case "state":
                        const entity = Entity.allEntities.get(entity_id);
                        if (entity) {
                            entity.setState(msg);
                        } else {
                            console.warn("received set message for entity", entity_id, "that isn't registered");
                        }
                        break;
                    case "config":
                        if (platform == "device") {
                            const dev = this.findOrUpdate(msg as any);
                            console.log("Device", dev);
                        } else {
                            console.warn("Unknown config msg");
                        }
                        break;
                    default:
                        console.warn("Ignoring subtopic", subtopic);
                }
            } else {
                console.log("<", line);
            }
        } catch (ex) {
            console.log("Error processing line", line, ex)
        }
    }

    sendLine = async (line: string) => {
        console.log(">", line);
        const encoder = new TextEncoder();
        const writer = this.port!.writable!.getWriter()
        await writer.write(encoder.encode(line + "\n"));
        writer.releaseLock();
    }

    setEntity = async(entityId: string, value: any) => {
        const ent = Entity.allEntities.get(entityId)

        let sval;
        if (value == null || value == undefined) {
            sval = "None"
        } else if (typeof(value) === 'object') {
            sval = JSON.stringify(value);
        } else {
            sval = value.toString();
        }

        await this.sendLine(`${ent?.platform}/${entityId}/set:${sval}`)
        
        // light/meche660d441571f4a2e_modbus1_26_coil/available:unavailable
    }

    resetDevice = async () => {
        const writer = this.port!.writable!.getWriter()
        await writer.write(new Uint8Array([3]));
        await new Promise((resolve, reject) => setTimeout(resolve, 200))
        await writer.write(new Uint8Array([4]));
        await new Promise((resolve, reject) => setTimeout(resolve, 200))
        writer.releaseLock();
    }

    requestDevices = async () => {
        console.log("Requesting ")
        await this.sendLine("status:online")
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
