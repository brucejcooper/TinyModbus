import { html, css, LitElement, PropertyValues } from "lit";
import { customElement, property, query, state } from "lit/decorators.js";
import { range } from "lit/directives/range.js";
import { map } from "lit/directives/map.js";
import { classMap } from "lit/directives/class-map.js";
import { SerialPort, Serial } from "./serial";
import { ModbusConnection } from "./ModbusConnection";
import { consume } from "@lit/context";
import { modbusContext } from "./ModbusConnectionContext";
import { SwitchFixtureID } from "./ModbusSwitchFixture";
import { when } from "lit/directives/when.js";
import { DaliProvisioner } from "./DaliProvisioner";
import { ExpandedDetail } from "./DaliLight";
import { repeat } from "lit/directives/repeat.js";

interface DaliDeviceInfo {
    address: number;
    manufacturer: string;
    serial: string;
    level: number;
    min: number;
    max: number;
    powerOnLevel: number;
    systemFailLevel: number;
    groups: number;
}


function arrayChanged(a: number[], b: number[]) {
    if (a === undefined) {
        return b !== undefined;
    }
    if (b === undefined) {
        return true;
    }
    if (a.length !== b.length) {
        return true;
    }
    for (let i = 0; i < a.length; i++) {
        if (a[i] !== b[i]) {
            return true;
        }
    }
    return false;
}

@customElement("modbus-dali-gateway")
export class ModbusDaliGateway extends LitElement {
    // ws?: WebSocket;

    @consume({ context: modbusContext })
    modbus: ModbusConnection = new ModbusConnection();

    @property()
    deviceId: number = 1;

    @state()
    daliProvisioner?: DaliProvisioner;

    @state({ hasChanged: arrayChanged })
    deviceLevels: number[] = [];


    @property()
    selectedDevice: string|undefined;


    statusFetcher: number = -1;


    @state()
    currentLight?: DaliDeviceInfo;




    createRenderRoot() {
        return this;
    }

    provisionStatusUpdated = () => { };

    provisionDaliReassign = () => {
        this.provisionDali(true);
    };

    provisionDaliUnassigned = () => {
        this.provisionDali(false);
    };

    private provisionDali = async (initAll: boolean) => {
        if (!this.modbus) {
            return;
        }
        if (this.daliProvisioner) {
            this.daliProvisioner.abort();
        }

        this.daliProvisioner = new DaliProvisioner(this.modbus);
        this.daliProvisioner.addEventListener("completed", () => {
            this.daliProvisioner = undefined;
        });
        await this.daliProvisioner.provision({ initAll, onUpdate: this.provisionStatusUpdated });
    };

    stopProvision = async () => {
        this.daliProvisioner?.abort();
    };

    startScan() {
        this.modbus?.startLongRunningProcess(0);
    }

    fetchStatus = async () => {
        if (this.modbus?.port?.connected && this.modbus.port.readable) {
            this.deviceLevels = await this.modbus?.getHoldingRegisters(this.deviceId, 0, 64);
        }
    }

    connectedCallback(): void {
        super.connectedCallback();
        this.modbus.addEventHandler("open", this.fetchStatus);
        this.statusFetcher = setInterval(this.fetchStatus, 10000) as unknown as number;
    }

    disconnectedCallback(): void {
        super.disconnectedCallback();
        this.modbus.removeEventHandler("open", this.fetchStatus);
        clearInterval(this.statusFetcher);
    }

    protected willUpdate(changed: PropertyValues): void {
        super.willUpdate(changed);
        if (changed.has("modbus")) {
            console.log("Modbus changed");
            this.fetchStatus();
        }
    }

    startDrag = (evt: Event) => {
        const dragEvent = evt as DragEvent;
        if (dragEvent.dataTransfer) {
            const src = evt.target as HTMLDivElement;
            const address = src.dataset.address;
            const content = `dali:${this.deviceId}/${address}`;
            dragEvent.dataTransfer.setData("modbus/device-address", content);
            dragEvent.dataTransfer.effectAllowed = "link";
        }
    };

    selectDevice = (evt: Event) => {
        const src = (evt.target as HTMLElement).closest(".dalidevice") as HTMLElement;
        const address = src.dataset.address;
        if (address) {
            const selection = `dali:${this.deviceId}/${address}`;
            this.dispatchEvent(new CustomEvent<string>("modbus-device-selected", { detail: selection, bubbles: true }))
        }

    }



    render() {

        console.log("Selected device", this.selectedDevice);

        const devices = map(range(64), (address) => {
            let light_status = this.deviceLevels[address] ?? 0xffff;
            if (light_status > 254) {
                return null;
            }

            const active = this.selectedDevice == `dali:${this.deviceId}/${address}`

            return html`<button type="button" data-bs-toggle="button" class=${classMap({
                btn: true, "btn-secondary": true, "btn-sm": true, dalidevice: true,
                active,
            })} draggable="true" data-address=${address} @click=${this.selectDevice}>
                ${address}
            </button>`;
        })


        return html`
            <div class="card">

                <div class="card-body">
                    <h5 class="card-title">DALI @${this.deviceId}</h5>

                <div style="display: inline-flex; flex-direction: row; gap: 0.5em;">
                    </div>

                <div class="switches" style="display: flex; flex-direction: row; gap: 1em; flex-wrap: wrap; margin-top: 1em;" @dragstart=${this.startDrag}>
                    ${devices}
                </div>

                  <div class="card-body text-end">
                    <button class="btn btn-primary" @click=${this.startScan}>Re-Scan</button>
                    ${when(this.daliProvisioner && this.daliProvisioner,
            () => html`<button class="btn btn-danger" @click=${() => this.daliProvisioner?.abort()}>Abort Commissioning</button>`,
            () => html`<span class="dropdown">
                                        <button class="btn btn-outline-secondary dropdown-toggle" type="button" data-bs-toggle="dropdown" aria-expanded="false">Commission</button>
                                        <ul class="dropdown-menu">
                                            <li><button class="dropdown-item" @click=${this.provisionDaliReassign}>Reassign all addresses</button></li>
                                            <li><button class="dropdown-item" @click=${this.provisionDaliUnassigned}>only unassigned</button></li>
                                        </ul>
                                   </span>`
        )}
                  </div>
                </div>
            </div>
        `;
    }
}

declare global {
    interface HTMLElementTagNameMap {
        "modbus-dali-gateway": ModbusDaliGateway;
    }
}
