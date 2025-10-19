import { html, css, LitElement, PropertyValues, TemplateResult } from "lit";
import { customElement, property, query, state } from "lit/decorators.js";
import { range } from "lit/directives/range.js";
import { map } from "lit/directives/map.js";
import { classMap } from "lit/directives/class-map.js";
import { ModbusConnection } from "./ModbusConnection";
import { consume } from "@lit/context";
import { modbusContext } from "./ModbusConnectionContext";
import { when } from "lit/directives/when.js";
import { DaliProvisioner } from "./DaliProvisioner";
import "./LocalStorageLabel";

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

@customElement("modbus-device")
export class ModbusDevice extends LitElement {

    @consume({ context: modbusContext })
    modbus!: ModbusConnection;

    @property({ type: Number})
    deviceId: number = 1;

    @state()
    daliProvisioner?: DaliProvisioner;

    @state({ hasChanged: arrayChanged })
    deviceLevels: number[] = [];


    @property({ type: Number})
    numAddresses: number = 32;


    @property()
    selectedDevice: string | undefined;

    @property()
    type: "dali" | "coils" | "inputs" = "dali";

    @state()
    currentLight?: DaliDeviceInfo;


    @state()
    dropTarget: number = -1;

    private pollHandlerFilter: { deviceId: number; } | undefined;

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
        await this.daliProvisioner.provision({ deviceId: this.deviceId, initAll, onUpdate: this.provisionStatusUpdated });
    };

    stopProvision = async () => {
        this.daliProvisioner?.abort();
    };

    startScan() {
        this.modbus?.startLongRunningProcess(0);
    }
    disconnectedCallback(): void {
        super.disconnectedCallback();
        this.removeEventHandler();
    }

    polledValuesChanged = () => {
        this.requestUpdate();
    }

    private removeEventHandler() {
        if (this.pollHandlerFilter) {
            this.modbus.removePollChangeHandler(this.polledValuesChanged, this.pollHandlerFilter);
        }
    }

    private refreshEventHandler() {
        this.removeEventHandler();
        this.pollHandlerFilter = { deviceId: this.deviceId };
        this.modbus.addPollChangeHandler(this.polledValuesChanged, this.pollHandlerFilter);
    }

    protected willUpdate(changed: PropertyValues): void {
        super.willUpdate(changed);

        if (this.modbus && (changed.has("deviceId") || changed.has("type"))) {
            this.refreshEventHandler();
            switch (this.type) {
                case 'coils':
                    this.deviceLevels = this.modbus.polledValues.get(this.deviceId)?.coils ?? [];
                    break;
                case 'dali':
                    this.deviceLevels = this.modbus.polledValues.get(this.deviceId)?.holding ?? [];
                    break;
                case 'inputs':
                    this.deviceLevels = this.modbus.polledValues.get(this.deviceId)?.inputs ?? [];
                    break;
            }
        }
    }

    startDrag = (evt: DragEvent) => {
        if (evt.dataTransfer) {
            const src = evt.target as HTMLDivElement;
            console.log("Drag", src);
            const address = src.dataset.address;
            const content = `${this.type}:${this.deviceId}/${address}`;
            evt.dataTransfer.setData("modbus/device-address", content);
            evt.dataTransfer.effectAllowed = "link";
            // src.classList.add("dragging");
        }
    };

    // endDrag = (evt: DragEvent) => {
    //     const src = evt.target as HTMLDivElement;
    //     src.classList.remove("dragging");
    // }

    selectDevice = (evt: Event) => {
        const src = (evt.target as HTMLElement).closest(".dalidevice") as HTMLElement;
        const address = src.dataset.address;
        if (address) {
            const selection = `${this.type}:${this.deviceId}/${address}`;
            this.dispatchEvent(new CustomEvent<string>("modbus-device-selected", { detail: selection, bubbles: true }))
        }
    }

    drop = (evt: Event) => {
        const data = (evt as DragEvent).dataTransfer?.getData("modbus/device-address");
        if (data) {
            // console.log("Binding Fixture", this.fixture, "button ", this.dropTargetButton, "to", data);
            const [type, deviceId, address] = data.split(/[:\/]/);
            const targetAddress = ((evt.target as HTMLElement).closest(".switchplate") as HTMLElement).dataset.address;
            const targetAddressInt = targetAddress == undefined ? undefined : parseInt(targetAddress);
            if (targetAddressInt !== undefined) {
                console.log("Drop", type, deviceId, address, "onto", targetAddress);
                // const address = parseInt(addressStr);
                // this.changeBindingAtIndex(this.dropTargetButton, [bus as "dali" | "modbus", address]);
                this.dropTarget = -1;
                const selection = `${this.type}:${this.deviceId}/${targetAddressInt - (targetAddressInt %7)}`;
                console.log("Selection", selection);
                this.dispatchEvent(new CustomEvent<string>("modbus-device-selected", { detail: selection, bubbles: true }))
            }

        }
    };

    dragover = (evt: Event) => {
        const ev = evt as DragEvent;
        const tgt = (evt.target as HTMLElement).closest(".switchplate") as HTMLElement;
        const addr = tgt?.dataset.address;
        if (addr) {
            ev.preventDefault();
            this.dropTarget = parseInt(addr);
            
            // this.dropTargetButton = idx == undefined ? -1 : Number(idx);
            if (ev.dataTransfer) {
                ev.dataTransfer.dropEffect = "link";
            }
        }
    };

    dragleave = (evt: Event) => {
        this.dropTarget = -1;
    };



    render() {
        let devices: any = null;
        

        if (this.type === 'inputs') { 
            const numVirtualDevices = Math.ceil(this.numAddresses / 7);

            devices = map(range(numVirtualDevices), (_, i) => {
                const address = i * 7; 
                const active = this.selectedDevice == `${this.type}:${this.deviceId}/${address}`
                let light_status = this.deviceLevels[address];
                const disabled = i > 5 && (light_status === undefined || light_status > 254);

                // return html`<modbus-switch-fixture deviceId=${this.deviceId} fixture=${fixtureNum} ></modbus-switch-fixture>`
                return html`<div style="align-items: center; display: flex; flex-direction: column">
                    <div class="rotate35"><localstorage-label key=${`modbus/label/${this.deviceId}/${address}`} placeholder=${`Plate ${i}`}></div>
                    <button class=${classMap({ switchfixture: true, dalidevice: true, active, })} @click=${this.selectDevice} data-address=${address} ?disabled=${disabled}>
                        ${map(range(5), (_, offset) => {
                            const btnaddr = address+offset;
                            const isOn = this.deviceLevels[btnaddr] > 0;
                            return html`<div class=${classMap({ switchplate: true, on: isOn, droptarget: this.dropTarget == btnaddr })} data-address=${btnaddr} @drop=${this.drop} @dragover=${this.dragover} @dragleave=${this.dragleave}></div>`;
                        })}
                    </button>
                </div>`

            })
        } else {
            devices = map(range(this.numAddresses), (address) => {
            const active = this.selectedDevice == `${this.type}:${this.deviceId}/${address}`
            let light_status = this.deviceLevels[address];
            const isOn = light_status && light_status < 255;
            const disabled = light_status === undefined || light_status > 254;

            return html`<button type="button" data-bs-toggle="button" class=${classMap({
                btn: true, "btn-secondary": !active, "btn-primary": active, "btn-sm": true, dalidevice: true,
                active,
            })} draggable="true" data-address=${address} @click=${this.selectDevice} ?disabled=${disabled}>
                <localstorage-label key=${`modbus/label/${this.deviceId}/${address}`} placeholder=${address}></localstorage-label>

                <i class=${classMap({ bi: true, "bi-lightbulb": light_status == 0, 'bi-lightbulb-off': disabled, 'bi-lightbulb-fill': isOn })}></i>
            </button>`;
        })
        }


        let actions: TemplateResult<1>[] = [];
        switch (this.type) {
            case 'dali':
                actions.push(html`<button class="btn btn-primary" @click=${this.startScan}>Re-Scan</button>`);
                if (this.daliProvisioner) {
                    actions.push(html`<button class="btn btn-danger" @click=${() => this.daliProvisioner?.abort()}>Abort Commissioning</button>`)
                } else {
                    actions.push(html`<span class="dropdown">
                        <button class="btn btn-outline-secondary dropdown-toggle" type="button" data-bs-toggle="dropdown" aria-expanded="false">Commission</button>
                        <ul class="dropdown-menu">
                            <li><button class="dropdown-item" @click=${this.provisionDaliReassign}>Reassign all addresses</button></li>
                            <li><button class="dropdown-item" @click=${this.provisionDaliUnassigned}>only unassigned</button></li>
                        </ul>
                    </span>`)
                }
        }


        return html`
            <div class="card">
                <div class="card-body" style="display: flex; flex-direction: row; gap: 1em;">
                    <h5 class="card-title">@${this.deviceId}</h5>
                    <div style="display: flex; flex-direction: row; gap: 0.5em; flex-wrap: wrap;" @dragstart=${this.startDrag}>
                        
                        ${devices}
                    </div>

                    ${when(actions.length > 0, () => html`<div class="card-body text-end">
                        ${actions}
                    </div>`)}
                    
                </div>
            </div>
        `;
    }
}

declare global {
    interface HTMLElementTagNameMap {
        "modbus-device": ModbusDevice;
    }
}
