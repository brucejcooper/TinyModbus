import { html, LitElement, PropertyValues } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { consume } from "@lit/context";
import { serialContext } from "./SerialContext";
import "./ModbusParameterEditor"
import { repeat } from "lit/directives/repeat.js";
import { Entity, HADevice, SerialConnection } from "./SerialConnection";
import { when } from "lit/directives/when.js";
import { live } from 'lit/directives/live.js';

@customElement("light-editor")
export class LightEditor extends LitElement {
    @consume({ context: serialContext })
    public serial!: SerialConnection;

    @property({ type: String, attribute: "device-id" })
    deviceId!: string;

    @state()
    device: HADevice | null = null;

    @state()
    label: string = "";

    @state()
    brightness: number = 0;


    protected createRenderRoot(): HTMLElement | DocumentFragment {
        return this;
    }


    entityChanged = (ent: Entity) => {
        console.log("Light entity changed", ent)
        this.requestUpdate();
    }

    protected willUpdate(changed: PropertyValues): void {
        super.willUpdate(changed);
        if (changed.has("deviceId")) {
            console.log("DeviceId changed")
            this.device = HADevice.allDevices.get(this.deviceId)!
        }

        const oldDevice = changed.get("device") as HADevice | undefined;
        if (oldDevice) {
            console.log("Changing out entity change listeners for ", oldDevice, this.device)
            oldDevice?.removeEntityChangeListener(this.entityChanged);
            this.device?.addEntityChangeListener(this.entityChanged);
        }
    }

    connectedCallback(): void {
        super.connectedCallback();
        if (this.deviceId) {
            this.device = HADevice.allDevices.get(this.deviceId)!;
            this.device.addEntityChangeListener(this.entityChanged);
        }
    }

    disconnectedCallback(): void {
        super.disconnectedCallback();
        this.device?.removeEntityChangeListener(this.entityChanged);
    }

    onOffChange = (evt: InputEvent) => {
        const inp = evt.target as HTMLInputElement
        this.serial.setEntity(this.device!.entities[0].id, { state: inp.checked ? "ON" : "OFF" });
    }

    brightnessChange = (evt: InputEvent) => {
        const inp = evt.target as HTMLInputElement
        this.serial.setEntity(this.device!.entities[0].id, { brightness: parseInt(inp.value) });
    }

    render() {
        if (!this.device) {
            return null;
        }
        const entity = this.device.entities[0];
        const value = entity.value;
        const brightness = value == undefined ? 0 : value.brightness ?? value.state === 'ON' ? 254 : 0;
        const disabled = false;

        return value?.brightness !== undefined ? html`
            <input id="inp" name="level" type="range" min="0" max="254" .value=${brightness?.toString()} ?disabled=${disabled} @change=${this.brightnessChange}/>
        `
            :
            html`<input id="inp" name="on/off" type="checkbox" ?checked=${value.state == "ON"} ?disabled=${disabled} @change=${this.onOffChange}/>`;
    }
}

declare global {
    interface HTMLElementTagNameMap {
        "light-editor": LightEditor;
    }
}
