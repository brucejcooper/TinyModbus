import { html, css, LitElement, TemplateResult } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { SerialPort, Serial } from "./serial";
import { ModbusConnection } from "./ModbusConnection";
import { provide } from "@lit/context";
import { modbusContext } from "./ModbusConnectionContext";
import { SwitchFixtureID } from "./ModbusSwitchFixture";
import { DaliProvisioner } from "./DaliProvisioner";
import "./ModbusDaliGateway"
import "./DaliLightEditor"


@customElement("modbus-app")
export class ModbusApplication extends LitElement {


    @provide({ context: modbusContext })
    modbus: ModbusConnection = new ModbusConnection();

    @state()
    serialInterface?: Serial | null;


    @property({ reflect: false })
    selectedBinding: SwitchFixtureID | null = null;

    @state()
    daliProvisioner?: DaliProvisioner;


    @property({ reflect: false })
    expandedDaliLight: number = -1;

    @state()
    selectedDevice: string|undefined;

    createRenderRoot() {
        return this;
    }

    startScan() {
        this.modbus?.startLongRunningProcess(0);
    }

    private async openPort(port: SerialPort) {
        console.log("Opening Port");
        this.modbus.open(port);

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
        this.modbus?.forget();
    }

    switchSelected = (evt: Event) => {
        this.selectedBinding = (evt as CustomEvent<SwitchFixtureID>).detail;
    };

    connectedCallback(): void {
        super.connectedCallback();
        this.addEventListener("switch-selected", this.switchSelected);
        this.serialInterface = (navigator as any).serial as Serial | undefined;
        if (this.serialInterface) {
            this.lookForBoundSerialInterface();
        } else {
            this.serialInterface = null;
        }
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

    setSelectedDevice = (evt: CustomEvent<string|undefined>) => {
        const sel = evt.detail;

        console.log("Received Selecting", this.selectedDevice);

        if (sel && sel === this.selectedDevice) {
            this.selectedDevice = undefined;
        } else {
            this.selectedDevice = sel;
        }
    }

    

    render() {
        if (this.serialInterface == null) {
            return html`<div class="alert alert-danger" role="alert">Serial not supported</div>`;
        }

        let deviceEditor: TemplateResult<1>|null|undefined = null;

        if (this.selectedDevice?.startsWith("dali:")) {
            const [deviceId,  address] = this.selectedDevice.substring(5).split('/').map(s => parseInt(s));
            console.log("Dali device", deviceId, "address", address, "for ", this.selectedDevice.substring(5).split('/').map(s => parseInt(s)))
            deviceEditor = html`<dali-light-editor .deviceId=${deviceId} .address=${address}></dali-light-editor>`;
        } else {
            deviceEditor = html`<div class="alert alert-primary" role="alert">
  Select a device to edit its properties here
</div>`;
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
                    ${this.modbus.port
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

                <div class="d-flex flex-column flex-grow-1 p-3 text-bg-tertiary" @modbus-device-selected=${this.setSelectedDevice}>  
                    <modbus-dali-gateway deviceId="3" .selectedDevice=${this.selectedDevice}></modbus-dali-gateway>
                </div>            

                <div class="" style=" width: 0.5rem; height: 100vh; flex-shrink: 0; background-color: #0000001a; border: solid rgba(0,0,0,.15); border-width: 1px 0; box-shadow: inset 0 .5em 1.5em #0000001a,inset 0 .125em .5em #00000026;"></div>

                <div class="d-flex flex-column flex-grow-0 p-3 text-bg-tertiary" style="width: 330px;"> 
                    ${deviceEditor}
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