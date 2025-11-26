import { html, LitElement, PropertyValues } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { classMap } from "lit/directives/class-map.js";
import { map } from "lit/directives/map.js";
import "./LocalStorageLabel";
import { DeviceSelection } from "./ModbusApp";
import { consume } from "@lit/context";
import { SerialConnection } from "./SerialConnection";
import { serialContext } from "./SerialContext";


@customElement("ha-switchfixture")
export class SwitchFixture extends LitElement {
    @consume({ context: serialContext })
    serialConnection!: SerialConnection;

    @property({ type: Number }) 
    fixture: number = 0;

    @property({ attribute: false }) 
    buttons: boolean[] = [false,false,false,false,false,false,false];

    @property({ type: Boolean }) 
    selected: number = 0;

    @state()
    dropTarget: number = -1;


    selectDevice(evt: MouseEvent) {
        this.dispatchEvent(new CustomEvent<DeviceSelection>("ha-device-selected", { detail: { device_id: 0, address: this.fixture }, bubbles: true}))
    }

    protected createRenderRoot(): HTMLElement | DocumentFragment {
        return this;
    }


    dragOver = (evt: DragEvent) => {
        const el = evt.target as HTMLElement;
        if (!el.dataset.index) {
            console.log(el);
            throw new Error("no index for element");
        }
        const idx = parseInt(el.dataset.index);
        this.dropTarget = idx;
        evt.preventDefault();
        if (evt.dataTransfer) {
            evt.dataTransfer.dropEffect = "link";
        }

    }
    dragLeave = (evt: DragEvent) => {
        this.dropTarget = -1;
    }
    drop = async (evt: DragEvent) => {
        this.dropTarget = -1;
        const el = evt.target as HTMLElement;
        if (!el.dataset.index) {
            console.log(el);
            throw new Error("no index for element");
        }
        const idx = parseInt(el.dataset.index);
        const address = this.fixture*8 + idx;


        const val = parseInt(evt.dataTransfer?.getData("ha/binding") ?? "0");
        if (val) {
            await this.serialConnection.sendLine(`bindings[${address}] = 0x${val.toString(16)}`)
        }
    }


    render() {
        const dropIndex = -1;

        return html`
                <div style="align-items: center; display: inline-flex; flex-direction: column; margin-top: 2em;" @click = ${this.selectDevice}>
                    <div class="rotate35"><localstorage-label key=${`ha/fixture${this.fixture}`} placeholder=${`fixture ${this.fixture}`}></localstorage-label></div>
                    <button 
                        class=${classMap({ 
                            switchfixture: true, 
                            dalidevice: true, 
                            active: this.selected, })} 
                    >

                        ${map(this.buttons, (on,idx) => {
                            return html`<div 
                                class=${classMap({ switchplate: true, on, droptarget: this.dropTarget == idx })} 
                                data-index=${idx}
                                @dragover=${this.dragOver}
                                @dragleave=${this.dragLeave}
                                @drop=${this.drop}
                                @click=${this.selectDevice}
                            >
                            </div>`;
                        })}
                    </button>
                </div>
            `;
    }
}




declare global {
    interface HTMLElementTagNameMap {
        "ha-switchfixture": SwitchFixture;
    }
}


