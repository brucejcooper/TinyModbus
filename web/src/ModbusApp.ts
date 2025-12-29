import { html, css, LitElement, TemplateResult } from "lit";
import { customElement, property, state } from "lit/decorators.js";
// import { SerialPort, Serial } from "./serial";
import { provide } from "@lit/context";
import { serialContext } from "./SerialContext";
import "./GatewayDevice"
import { SerialConnection } from "./SerialConnection";
import { GatewayDeviceState } from "./deviceState";
import { renderBindingsEditor } from "./FixtureEditor";
import "./DaliLightEditor"
import "./ModbusEditor"
import { loadMappings, NameMappings } from "./names";

export type DeviceSelection = null|{
    device_id: number;
    address: number
};

function editorForEntity(sel: DeviceSelection, gw: GatewayDeviceState|null ): TemplateResult<1>|null {
    if (!sel || !gw) {
        return null;
    }

    switch (sel.device_id) {
        case 0:
            return renderBindingsEditor(sel.address, gw.bindings.slice(sel.address*8, sel.address*8+7));
            
        case 2: 
        case 3:
            const gwdev = gw.dali[sel.device_id];
            return html`<ha-dali-light-editor .light=${gwdev ? gwdev[sel.address] : null} .deviceid=${sel.device_id} .address=${sel.address} ></ha-dali-light-editor>`;
        case 1:
            return html`<ha-modbus-editor .deviceid=${sel.device_id} .address=${sel.address} ></ha-modbus-editor>`;
        default:
            return html`<div class="alert alert-warning" role="alert">
                No Editor for device ${sel.device_id}/${sel.address}
            </div>`;
    }

}



@customElement("modbus-app")
export class ModbusApplication extends LitElement {

    @provide({ context: serialContext })
    serialConnection: SerialConnection = new SerialConnection();

    @state()
    gatewayDevice: GatewayDeviceState|null = null;

    @state()
    selectedDevice: DeviceSelection = null;


    @state() 
    mappings: NameMappings = {};

    // @state()
    // serialInterface?: Serial | null;

    @state() 
    deviceLog: string[] = [];
    scrollTimer?: number;


    createRenderRoot() {
        return this;
    }


    async disconnectSerial() {
        this.serialConnection?.forget();
    }

    // switchSelected = (evt: Event) => {
    //     // this.selectedBinding = (evt as CustomEvent<string>).detail;
    // };

    topLevelDevicesChanged = (dev:GatewayDeviceState) => {
        this.gatewayDevice = dev;
    }

    connectedCallback(): void {
        super.connectedCallback();
        // this.addEventListener("switch-selected", this.switchSelected);
        this.serialConnection.open();

        this.serialConnection.addChangeListener(this.topLevelDevicesChanged);
        this.serialConnection.addConsoleListener(this.consoleLogReceived);

        loadMappings().then(() => {
            this.requestUpdate();
        });
    }

    disconnectedCallback(): void {
        super.disconnectedCallback();
        this.serialConnection.removeChangeListener(this.topLevelDevicesChanged);
        this.serialConnection.removeConsoleListener(this.consoleLogReceived);

        // this.removeEventListener("switch-selected", this.switchSelected);
    }

    deviceSelected = (evt: CustomEvent<DeviceSelection>) => {        
        this.selectedDevice = evt.detail;
        // this.device =  this.selectedDevice ? HADevice.allDevices.get(this.selectedDevice) ?? null : null;
    }

    consoleInputEntered = (evt: Event) => {
        const input = (evt.target as HTMLInputElement)
        this.serialConnection.sendLine(input.value);
        input.value = "";
    }

    consoleLogReceived = (line: string) => {
        if (this.deviceLog.length > 1000) {
            this.deviceLog.shift();
        }
        this.deviceLog.push(line + "\n");
        this.requestUpdate();
        if (this.scrollTimer) {
            clearTimeout(this.scrollTimer);
        }
        this.scrollTimer = setTimeout(() => {
            this.querySelector("#log")?.scrollTo(0, 1000000000);
        }, 100) as any;
        
    }

    sendFile = () => {
        console.log("Uploading file");
        this.serialConnection.sendLine("upload");
    }


    sendBindings = () => {
        console.log("Uploading Bindings");
        this.serialConnection.sendLine("upload_bindings");
    }

    

    render() {
        // if (this.serialInterface == null) {
        //     return html`<div class="alert alert-danger" role="alert">Serial not supported</div>`;
        // }

        return html`
            <main class="d-flex flex-nowrap" @ha-device-selected=${this.deviceSelected}>
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

                    
                    <button class="btn btn-secondary" @click=${this.sendFile} type="button">Upload script</button>
                    <button class="btn btn-secondary" @click=${this.sendBindings} type="button">Upload Bindings</button>


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

                <div class="d-flex flex-column flex-grow-1 p-3 text-bg-tertiary" style="gap: 1em; overflow-y: auto; max-height: 100vh" >  
                    <ha-gw-device .state=${this.gatewayDevice} .selectedDevice=${this.selectedDevice}></ha-gw-device>
                </div>            

                <div class="" style=" width: 0.5rem; height: 100vh; flex-shrink: 0; background-color: #0000001a; border: solid rgba(0,0,0,.15); border-width: 1px 0; box-shadow: inset 0 .5em 1.5em #0000001a,inset 0 .125em .5em #00000026;"></div>

                <div class="d-flex flex-column flex-grow-0 p-3 text-bg-tertiary" style="min-width: 380px; height: 100vh;"> 
                    <div style="flex:1; overflow-y: auto;">
                        ${editorForEntity(this.selectedDevice, this.gatewayDevice)}
                    </div>
                    <div style="flex:0">
                        <h4>Console</h4>
                        <input type="text" style="width: 100%" @change=${this.consoleInputEntered}/>
                    </div>
                    <pre id="log" style="border: 1px solid #aaa; max-width: 380px; min-height: 5em; flex: 0.4; white-space: perw-wrap;">${this.deviceLog}</pre>
                </div>            
            </main>
        `;
    }
}

// declare global {
//     interface HTMLElementTagNameMap {
//         "modbus-app": ModbusApplication;
//     }
// }