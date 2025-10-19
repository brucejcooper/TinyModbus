import { html, LitElement } from "lit";
import { customElement, property } from "lit/decorators.js";
import { ModbusConnection } from "./ModbusConnection";
import { consume } from "@lit/context";
import { modbusContext } from "./ModbusConnectionContext";
import "./ModbusParameterEditor"
import { repeat } from "lit/directives/repeat.js";
import { range } from "lit/directives/range.js";

export interface ExpandedDetail {
    address: number;
    expanded: boolean;
}

@customElement("modbus-input-bindings-editor")
export class ModbusInputBindingsEditor extends LitElement {
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

    clearBinding = (evt: MouseEvent) => {
        evt.stopPropagation();
        evt.preventDefault();
    }


    render() {
        return html`
        <div class="bindingseditorxxx">
            <div class="fs-4">Modbus ${this.deviceId} Fixture ${this.address/7}</div> 


            <localstorage-label key=${`modbus/label/${this.deviceId}/${this.address}`} editable label="Label"></localstorage-label>



            ${repeat(range(5), (_, idx) => html`
                <div class="input-group mb-3">
                    <div class="form-floating">
                        <input type="text" class="form-control" readonly id="btn${idx}" placeholder="Binding" value="foo">
                        <label for="btn1">Binding${idx}</label>
                    </div>
                    <button class="input-group-text"><i class="bi bi-x"></i></button>
                </div>
            `)}

        </div>
        `;
    }
}

declare global {
    interface HTMLElementTagNameMap {
        "modbus-input-bindings-editor": ModbusInputBindingsEditor;
    }
}
