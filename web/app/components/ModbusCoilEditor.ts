import { html, css, LitElement, PropertyValues } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { ModbusConnection } from "./ModbusConnection";
import { consume } from "@lit/context";
import { modbusContext } from "./ModbusConnectionContext";
import { classMap } from "lit/directives/class-map.js";
import "./ModbusParameterEditor"
import { ModbusParameterEditor } from "./ModbusParameterEditor";

export interface ExpandedDetail {
    address: number;
    expanded: boolean;
}

@customElement("modbus-coil-editor")
export class ModbusCoilEditor extends LitElement {
    @consume({ context: modbusContext })
    public modbus?: ModbusConnection;

    @property({ type: Number })
    deviceId: number = 1;


    @property({ type: Number })
    address: number = -1;

    @property({ reflect: false })
    label: string = "";

    protected createRenderRoot(): HTMLElement | DocumentFragment {
        return this;
    }

    connectedCallback(): void {
        super.connectedCallback();
        this.label = localStorage.getItem(`dali${this.address}.label`) ?? `Dali${this.address}`;
    }

    toggle = async () => {
        await this.modbus?.setCoil(this.deviceId, this.address, "toggle");
    }


    render() {
        return html`
        <div class="lighteditor">
            <div class="fs-4">Modbus device ${this.deviceId} coil ${this.address}</div> 

            <localstorage-label key=${`modbus/label/${this.deviceId}/${this.address}`} editable label="Label" placeholder=${this.address}></localstorage-label>


            <form class="details">
                <label for="coil_button" class="form-label">On</label>
                <modbus-parameter-editor id="coil_button" label="On" .fetchCmd=${0x1a0} .setCmd=${-1} .deviceId=${this.deviceId} .address=${this.address}></modbus-parameter-editor>
            </form>
        </div>
        `;
    }
}

declare global {
    interface HTMLElementTagNameMap {
        "modbus-coil-editor": ModbusCoilEditor;
    }
}
