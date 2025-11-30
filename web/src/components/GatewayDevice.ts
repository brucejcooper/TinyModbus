import { html, LitElement, TemplateResult } from "lit";
import { customElement, property } from "lit/decorators.js";
import { repeat } from "lit/directives/repeat.js";
import { range } from "lit/directives/range.js";
import { when } from "lit/directives/when.js";
import { consume } from "@lit/context";
import "./LocalStorageLabel"
import "./DaliLight"
import "./Fixture"
import { GatewayDeviceState } from "../deviceState";
import "./Fixture";
import { DeviceSelection } from "./ModbusApp";
import { serialContext } from "./SerialContext";
import { SerialConnection } from "./SerialConnection";


@customElement("ha-gw-device")
export class GatewayDeviceView extends LitElement {
    @property({ attribute: false })
    state: GatewayDeviceState|null = null;


    @consume({ context: serialContext })
    serialConnection!: SerialConnection;
    
    @property({attribute: false})
    public selectedDevice: DeviceSelection = null;

    
    protected createRenderRoot(): HTMLElement | DocumentFragment {
        return this;
    }

    selectDevice(evt: MouseEvent) {
        // this.dispatchEvent(new CustomEvent<string|null>("ha-device-selected", { detail: this.deviceId, bubbles: true }))
        evt.stopPropagation();
    }

    async provision(evt: MouseEvent) {
        const el = (evt.target as HTMLButtonElement);
        const device_id = parseInt(el.dataset.device ?? "0")
        const full = el.dataset.type == "full"

        await this.serialConnection.sendLine(`dali[${device_id}]:provision(${full})`);

    }

    renderUnavailable = () => {
        return html`
            <div class="alert alert-warning" role="alert">Unavailable</div>
        `
    }


    render() {
        if (!this.state) {
            return html``;
        }
        return html`<div class="device" @click=${this.selectDevice}>
            <h3>Switches</h3>

            <div style="display: flex; flex-direction: row; gap: 0.2em; flex-wrap: wrap;">
                ${repeat(range(24), (i)=>i, (fixturenum) => html`<ha-switchfixture .fixture=${fixturenum} .buttons=${this.state!.buttons.slice(fixturenum*8, (fixturenum*8)+7) ?? []} ?selected=${this.selectedDevice?.device_id == 0 && this.selectedDevice?.address == fixturenum}></ha-switchfixture>`)}
            </div>

            ${repeat(Object.entries(this.state!.modbus), ([device_id_str,dev])=> device_id_str, ([device_id_str,relays]) => html`
                <h3>Modbus GW ${device_id_str}</h3>
                ${when(relays == null, this.renderUnavailable, () => {
                    const device_id = parseInt(device_id_str);
                    const deviceAvailable = this.selectedDevice?.device_id == device_id;
                    return html`
                    <div style="display: flex; flex-direction: row; gap: 0.2em; flex-wrap: wrap;">
                        ${repeat(relays!, (l, addr) => addr, (relay, addr) => html`
                            <ha-dali-light 
                                .on=${relay} 
                                .disabled=${false}
                                .device_id=${device_id} 
                                .selected=${deviceAvailable && this.selectedDevice?.address == addr }
                                .address=${addr}>
                            </ha-dali-light>
                        `)}
                    </div>
                `})}
            `)}

                ${repeat(Object.entries(this.state!.dali), ([device_id_str,_])=> device_id_str, ([device_id_str,daliLights]) => {
                const device_id = parseInt(device_id_str);
                const deviceAvailable = this.selectedDevice?.device_id == device_id;

                return html`
                    <h3>Dali GW ${device_id}</h3>
                    ${when(daliLights == null, this.renderUnavailable, () => html`
                        <div style="margin-bottom: 2em">
                            <button class="btn btn-primary" data-device=${device_id} data-type="full" @click=${this.provision}>Provision</button>
                            <button class="btn btn-secondary" data-device=${device_id} data-type="incremental" @click=${this.provision}>Incremental Provision</button>
                        </div>

                        <div style="display: flex; flex-direction: row; gap: 0.2em; flex-wrap: wrap;">
                            ${repeat(daliLights!, (l, addr) => addr, (light, addr) => html`
                                <ha-dali-light 
                                    .on=${light?.brightness ?? 0 > 0} 
                                    .disabled=${light == null}
                                    .device_id=${device_id} 
                                    .selected=${deviceAvailable && this.selectedDevice?.address == addr }
                                    .address=${addr}>
                                </ha-dali-light>
                            `)}
                        </div>
                    `)}
                `})}
        </div>`;
    }
}




declare global {
    interface HTMLElementTagNameMap {
        "ha-gw-device": GatewayDeviceView;
    }
}
