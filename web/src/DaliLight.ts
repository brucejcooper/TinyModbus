import { html, LitElement, PropertyValues } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { classMap } from "lit/directives/class-map.js";
import "./LocalStorageLabel"
import { DeviceSelection } from "./ModbusApp";
import { consume } from "@lit/context";
import { serialContext } from "./SerialContext";
import { SerialConnection } from "./SerialConnection";




@customElement("ha-dali-light")
export class DaliLightView extends LitElement {


    // @property({ attribute: false})
    // light: DaliLight|null = null;

    @property({ attribute: false})
    device_id: number = 0;

    @property({ attribute: false})
    address: number = 0;

    @property({ attribute: false})
    on: boolean = false;

    @property({ attribute: false})
    disabled: boolean = false;

    @property({ attribute: false})
    selected: boolean = false;

    @consume({ context: serialContext })
    serialConnection!: SerialConnection;
    

    
    protected createRenderRoot(): HTMLElement | DocumentFragment {
        return this;
    }

    startDrag(evt: DragEvent) {
        if (evt.dataTransfer) {
            const content = this.device_id << 8 | this.address;
            evt.dataTransfer.setData("ha/binding", content.toString());
            evt.dataTransfer.effectAllowed = "link";
            // src.classList.add("dragging");
        }
    }

    selectDevice(evt: MouseEvent) {
        if (this.device_id == 1) { 
            this.serialConnection.sendLine(`modbus[${this.device_id}][${this.address}] = not modbus[${this.device_id}][${this.address}]`)
        }
        this.dispatchEvent(new CustomEvent<DeviceSelection>("ha-device-selected", { detail: { device_id: this.device_id, address: this.address}, bubbles: true }))
        evt.stopPropagation();
    }



    render() {
        return html`<button type="button"  class=${classMap({
                btn: true, 
                "m-1": true,
                "btn-secondary": !this.selected, 
                "btn-primary": this.selected, 
                "btn-sm": true, 
            })} draggable="true" data-address=${this.id} @click=${this.selectDevice} ?disabled=${this.disabled} @dragstart=${this.startDrag}>
                <localstorage-label .deviceId=${this.device_id} .address =${this.address} placeholder=${`${this.address}`}></localstorage-label>

                <i class=${classMap({ bi: true, "bi-lightbulb": !this.on && !this.disabled, 'bi-lightbulb-off': this.disabled, 'bi-lightbulb-fill': this.on })}></i>
            </button>`;
    }
}


// declare global {
//     interface HTMLElementTagNameMap {
//         "ha-dalilight": DaliLightView;
//     }
// }
