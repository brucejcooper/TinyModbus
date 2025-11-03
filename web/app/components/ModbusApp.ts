import { html, css, LitElement, TemplateResult, PropertyValues } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { SerialPort, Serial } from "./serial";
import { provide } from "@lit/context";
import { serialContext } from "./SerialContext";
import { SwitchFixtureID } from "./ModbusSwitchFixture";
import "./LightEditor"
import "./Device"
import "./entities/BinarySensor"
import "./entities/Text"
import "./entities/Light"
import "./entities/Button"
import "./entities/Sensor"
import { SerialConnection, HADevice, Entity } from "./SerialConnection";
import { repeat } from "lit/directives/repeat.js";
import { selectedDeviceContext } from "./SelectedDeviceContext";


function stringListChanged(a: string[], b: string[]) {
    if (Array.isArray(a) && !Array.isArray(b)) {
        return true;
    }
    if (Array.isArray(b) && !Array.isArray(a)) {
        return true;
    }
    if (!a) {
        return !!b;
    }
    if (a.length !== b.length)
        return true;

    for (let i = 0; i < a.length; i++) {
        if (a[i] !== b[i]) {
            return true;
        }
    }
    return false;
}

function editorForEntity(ent: Entity): TemplateResult<1>|null {
    if (!ent) {
        return null;
    }

    switch (ent.platform) {
        case "binary_sensor": 
            return html`<entity-binary-sensor .entity=${ent}></entity-binary-sensor>`
        case "text": 
            return html`<entity-text .entity=${ent}></entity-text>`
        case "light": 
            return html`<entity-light .entity=${ent}></entity-light>`
        case "button": 
            return html`<entity-button .entity=${ent}></entity-button>`
        case "sensor": 
            return html`<entity-sensor .entity=${ent}></entity-sensor>`
        default:
            return html`<div class="alert alert-warning" role="alert">
                ${ent.platform} ${ent.id}
            </div>`;
    }
}



@customElement("modbus-app")
export class ModbusApplication extends LitElement {


    @provide({ context: serialContext })
    serialConnection: SerialConnection = new SerialConnection();

    @provide({ context: selectedDeviceContext, })
    selectedDevice: string|null = null;


    @state()
    serialInterface?: Serial | null;


    @property({ reflect: false })
    selectedBinding: SwitchFixtureID | null = null;

    @property({ reflect: false })
    expandedDaliLight: number = -1;


    @state({ hasChanged: stringListChanged })
    topLevelDeviceIDs: string[] = []

    @state()
    device: HADevice|null = null;

    createRenderRoot() {
        return this;
    }

    startScan() {
        // this.modbus?.startLongRunningProcess(0);
    }

    private async openPort(port: SerialPort) {
        try {
            await this.serialConnection.open(port);
            // await this.serialConnection.requestDevices();
        } catch (ex) {
            console.log(ex);
        }
    }

    async lookForBoundSerialInterface() {

        if (!this.serialInterface) {
            return;
        }
        const ports = await this.serialInterface.getPorts();
        console.log("Looking for serial interface", this.serialInterface, ports);
        if (ports.length > 0) {
            this.openPort(ports[0]);
        }
    }

    async requestSerial() {
        const port = await this.serialInterface?.requestPort();
        if (port) {
            this.openPort(port);
        }
    }

    async disconnectSerial() {
        this.serialConnection?.forget();
    }


    switchSelected = (evt: Event) => {
        this.selectedBinding = (evt as CustomEvent<SwitchFixtureID>).detail;
    };

    topLevelDevicesChanged = (dev:HADevice) => {
        this.topLevelDeviceIDs = this.serialConnection.child_devices.map(e => e.id);
    }

    connectedCallback(): void {
        super.connectedCallback();
        this.addEventListener("switch-selected", this.switchSelected);
        this.serialInterface = (navigator as any).serial as Serial | undefined;
        if (this.serialInterface) {
            this.lookForBoundSerialInterface();
        } else {
            this.serialInterface = null;
        }

        this.serialConnection.addChangeListener(this.topLevelDevicesChanged);
    }

    disconnectedCallback(): void {
        super.disconnectedCallback();
        this.serialConnection.removeChangeListener(this.topLevelDevicesChanged);
        this.removeEventListener("switch-selected", this.switchSelected);
    }

    deviceSelected = (evt: CustomEvent<string|null>) => {        
        this.selectedDevice = evt.detail;
        this.device =  this.selectedDevice ? HADevice.allDevices.get(this.selectedDevice) ?? null : null;

        // this.requestUpdate();
    }

    
    stopProvision = async () => {
        // this.daliProvisioner?.abort();
    };

    

    render() {
        if (this.serialInterface == null) {
            return html`<div class="alert alert-danger" role="alert">Serial not supported</div>`;
        }

        

        return html`
            <main class="d-flex flex-nowrap">
                <nav class="nav d-flex flex-column flex-shrink-0 p-3 text-bg-dark" style="width: 280px;"> 
                    <a href="/" class="d-flex align-items-center mb-3 mb-md-0 me-md-auto text-white text-decoration-none"> 
                        <svg class="bi pe-none me-2" width="40" height="32" aria-hidden="true"><use xlink:href="#bootstrap"></use></svg> 
                        <span class="fs-4">Modbus</span> 
                    </a> 
                    <hr> 
                    <ul class="nav nav-pills flex-column mb-auto"> 
                        <li class="nav-item"> 
                            <a href="#" class="nav-link active" aria-current="page"> 
                                <svg class="bi pe-none me-2" width="16" height="16" aria-hidden="true"><use xlink:href="#home"></use></svg>
                                Devices
                            </a> 
                        </li> 

                    </ul> 
                    ${this.serialConnection.port
                        ? html`<button class="btn btn-secondary" @click=${this.disconnectSerial} type="button">Disconnect</button>`
                        : html`<button class="btn btn-secondary" @click=${this.requestSerial} type="button">Connect</button>`}

                    <hr> 

                    <div class="dropdown"> 
                        <a href="#" class="d-flex align-items-center text-white text-decoration-none dropdown-toggle" data-bs-toggle="dropdown" aria-expanded="false"> 
                            <img src="https://github.com/mdo.png" alt="" width="32" height="32" class="rounded-circle me-2"> <strong>bruce</strong> 
                        </a> 
                        <ul class="dropdown-menu dropdown-menu-dark text-small shadow"> 
                            <li><a class="dropdown-item" href="#">New project...</a></li> 
                            <li><a class="dropdown-item" href="#">Settings</a></li> 
                            <li><a class="dropdown-item" href="#">Profile</a></li> 
                            <li><hr class="dropdown-divider"></li> 
                            <li><a class="dropdown-item" href="#">Sign out</a></li> 
                        </ul> 
                    </div> 
                </nav>

                <div class="d-flex flex-column flex-grow-1 p-3 text-bg-tertiary" style="gap: 1em; overflow-y: auto; max-height: 100vh" @ha-device-selected=${this.deviceSelected}>  
                    ${repeat(this.topLevelDeviceIDs, (id) => html`<ha-device device-id=${id}></ha-device>`)}
                </div>            

                <div class="" style=" width: 0.5rem; height: 100vh; flex-shrink: 0; background-color: #0000001a; border: solid rgba(0,0,0,.15); border-width: 1px 0; box-shadow: inset 0 .5em 1.5em #0000001a,inset 0 .125em .5em #00000026;"></div>

                <div class="d-flex flex-column flex-grow-0 p-3 text-bg-tertiary" style="min-width: 380px;"> 
                    <h3>Device ${this.device?.name ?? this.device?.id ?? "none"}</h3>
                    <div class="keyvalgrid">
                    ${repeat((this.device?.entities ?? []), e => html`<div class="noshrink">${e.name ?? e.id}:</div>${editorForEntity(e)}`)}
                    </div>
                </div>            
            </main>
        `;
    }
}

declare global {
    interface HTMLElementTagNameMap {
        "modbus-app": ModbusApplication;
    }
}