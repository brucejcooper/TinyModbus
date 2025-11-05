import { html, css, LitElement, PropertyValues } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { repeat } from "lit/directives/repeat.js";
import { consume } from "@lit/context";
import { serialContext } from "./SerialContext";
import { selectedDeviceContext } from "./SelectedDeviceContext";
import { SerialConnection, HADevice, Entity } from "./SerialConnection";
import { classMap } from "lit/directives/class-map.js";
import { map } from "lit/directives/map.js";
import "./LocalStorageLabel"


function renderDevice(d: HADevice) {
    const id = d.id

    if (d.name.startsWith("Switchplate ")) {
        return html`<ha-fixture device-id=${id}></ha-fixture>`;        
    } else if (d.name.startsWith("Light ")) {
        return html`<ha-light device-id=${id}></ha-light>`;        
    }
    
    return html`<ha-device device-id=${id}></ha-device>`;
}



@customElement("ha-device")
export class DeviceView extends LitElement {
    @consume({ context: serialContext })
    public serialConnection!: SerialConnection;

    @consume({ context: selectedDeviceContext, subscribe: true })
    public selectedDevice!: string|null;


    @property({ type: String, attribute: "device-id" })
    deviceId?: string
    
    @state()
    device: HADevice | null = null;

    protected createRenderRoot(): HTMLElement | DocumentFragment {
        return this;
    }

    selectDevice(evt: MouseEvent) {
        this.dispatchEvent(new CustomEvent<string|null>("ha-device-selected", { detail: this.deviceId, bubbles: true }))
        evt.stopPropagation();
    }

    protected willUpdate(changed: PropertyValues): void {
        super.willUpdate(changed);

        if (changed.has("deviceId")) {
            if (this.device) {
                this.device.removeChangeListener(this.deviceChanged);
                this.device.removeEntityChangeListener(this.entityChanged);
            }
            this.device = this.deviceId ? HADevice.allDevices.get(this.deviceId) ?? null : null;
            this.device?.addChangeListener(this.deviceChanged);
            this.device?.addEntityChangeListener(this.entityChanged);
        }
    }

    entityChanged = (entity: Entity) => {
        this.requestUpdate();
    }


    deviceChanged = () => {
        this.requestUpdate();
    }

    disconnectedCallback(): void {
        super.disconnectedCallback();
        if (this.device) {
            this.device.removeChangeListener(this.deviceChanged);
            this.device.removeEntityChangeListener(this.entityChanged);
        }
    }


    render() {
        const device = this.device;
        if (!device) {
            return html`No Device`;
        }
        const childDevices = [...device.child_devices];

        return html`<div class="device" @click=${this.selectDevice}>
            <h3>${device.name}</h3>

             <div style="padding-left: 2em">
                ${repeat(childDevices, d => d.id, renderDevice)}
            </div>
        </div>`;
    }
}



@customElement("ha-fixture")
export class FixtureDeviceView extends DeviceView {
    @state()
    dropTarget: string|null = null;

    render() {
        const device = this.device;
        if (!device) {
            return html`No Device`;
        }

        const active = this.selectedDevice == this.deviceId;
        const address = 0;
        const disabled = false;

        const buttonEntities = device.entities.filter(e => e.platform == 'binary_sensor')
        return html`
            <div style="align-items: center; display: inline-flex; flex-direction: column; margin-top: 2em;" >
                <div class="rotate35"><localstorage-label key=${`ha/label/${device.id}`} placeholder=${device.name}></div>
                <button 
                    class=${classMap({ 
                        switchfixture: true, 
                        dalidevice: true, 
                        active, })} 
                    @click=${this.selectDevice} 
                    data-address=${address} 
                    ?disabled=${disabled}>

                    ${map(buttonEntities, e => {
                        return html`<div class=${classMap({ switchplate: true, on: e.is_on(), droptarget: this.dropTarget == e.id })} data-entityid=${e.id}></div>`;
                    })}
                </button>
            </div>
        `;
    }
}

@customElement("ha-light")
export class LightView extends DeviceView {

    @state()
    modbusDeviceId: number = 0;


    @state()
    address: number = 0;

    startDrag(evt: DragEvent) {
        if (evt.dataTransfer) {
            const lightEntity = HADevice.allDevices.get(this.deviceId!)?.entities[0]
            if (lightEntity) {
                const content = lightEntity.id;
                evt.dataTransfer.setData("ha/entityid", content);
                evt.dataTransfer.effectAllowed = "link";
                // src.classList.add("dragging");
            }
        }
    }


    protected willUpdate(changed: PropertyValues): void {
        super.willUpdate(changed);

        if (changed.has("deviceId") ) {
            if (this.deviceId) {
                const m = this.deviceId.match(/(modbus|daligw)(?<did>\d+)_(?<add>\d+)$/)
                if (m) {                    
                    this.modbusDeviceId = parseInt(m.groups!.did)
                    this.address = parseInt(m.groups!.add)
                }
            } else {
                this.modbusDeviceId = 0;
                this.address = 0;
            }
        }
    }

    render() {
        const device = this.device;
        if (!device) {
            return html`No Device`;
        }
        const entity = device.entities.values().next().value!;

        const disabled = false;
        const isOn = entity.is_on();
        const isSelected = this.selectedDevice == this.deviceId;

        return html`<button type="button"  class=${classMap({
                btn: true, 
                "m-1": true,
                "btn-secondary": !isSelected, 
                "btn-primary": isSelected, 
                "btn-sm": true, 
            })} draggable="true" data-address=${this.id} @click=${this.selectDevice} ?disabled=${disabled} @dragstart=${this.startDrag}>
                <localstorage-label key=${`ha/label/${device.id}`} placeholder=${device.name}></localstorage-label>

                <i class=${classMap({ bi: true, "bi-lightbulb": !isOn, 'bi-lightbulb-off': disabled, 'bi-lightbulb-fill': isOn })}></i>
            </button>`;
    }
}




declare global {
    interface HTMLElementTagNameMap {
        "ha-device": DeviceView;
        "ha-fixture": FixtureDeviceView;
        "ha-light": LightView;
    }
}
