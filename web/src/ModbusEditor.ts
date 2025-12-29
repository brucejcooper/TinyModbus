import { html, LitElement, TemplateResult } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { consume } from "@lit/context";
import "./LocalStorageLabel"
import "./DaliLight"
import "./Fixture"
import { serialContext } from "./SerialContext";
import { SerialConnection } from "./SerialConnection";

interface UpdateDetail {
    attribute: string;
    value: number;
}




@customElement("ha-modbus-editor")
export class ModbusEditor extends LitElement {

    @consume({ context: serialContext })
    serialConnection!: SerialConnection;


    @property({attribute: false})
    deviceid!: number;

    @property({attribute: false})
    address!: number;

    
    protected createRenderRoot(): HTMLElement | DocumentFragment {
        return this;
    }


    valueChanged = (evt:CustomEvent<UpdateDetail>) => {
        const { attribute, value } = evt.detail;
        this.serialConnection.sendLine(`dali[${this.deviceid}][${this.address}].${attribute}=${value}`)
        evt.stopPropagation();
    }

    render() {
        return html`
            <h3>Dali light ${this.deviceid}/${this.address}</h3>

            <localstorage-label .deviceId=${this.deviceid} .address =${this.address} label="label" ?editable=${true} placeholder=${`${this.address}`}></localstorage-label>
        `;
    }
}