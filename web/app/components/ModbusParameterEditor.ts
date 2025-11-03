import { html, css, LitElement, PropertyValues } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { repeat } from "lit/directives/repeat.js";
import { consume } from "@lit/context";
import { serialContext } from "./SerialContext";
import { range } from "lit/directives/range.js";
import { classMap } from "lit/directives/class-map.js";
import { SerialConnection } from "./SerialConnection";


export interface ExpandedDetail {
    address: number;
    expanded: boolean;
}

@customElement("modbus-parameter-editor")
export class ModbusParameterEditor extends LitElement {
    static styles = css`
    :host {
        display: inline-flex;
    }

    input {
        width: 100%;
    }
    `;

    @consume({ context: serialContext })
    public modbus?: SerialConnection;

    @property({ type: Number })
    deviceId: number = -1;

    @property({ type: Number })
    address: number = -1;

    @property()
    label: string = "unknown";

    @property()
    disabled: boolean = false;

    @property({ type: Number })
    fetchCmd?: number;

    @property({ type: Number })
    setCmd?: number;

    @property({ reflect: false })
    value: number | undefined;

    @property({ reflect: false })
    error: any = null;

    @property()
    type: "slider" | "bits" | "coil" = "slider";

    private abortCtrl?: AbortController;

    fetch = async () => {
        if (this.abortCtrl) {
            this.abortCtrl.abort();
        }
        this.abortCtrl = new AbortController();
        const signal = this.abortCtrl.signal;
        this.error = null;
        try {
            if (this.modbus && this.deviceId > 0 && (this.address !== -1)) {
                let val: number|undefined;
                // if (this.type == "coil") {
                //     // Just fetch the single coil value. 
                //     val = Number(await this.modbus.getCoils(this.deviceId, this.address, 1));
                // } else {
                //     if (this.fetchCmd) {
                //         val = await this.modbus.executeCustomDaliCommand(this.deviceId, this.fetchCmd | (this.address << 9));
                //     }
                // }
                if (!signal.aborted) {
                    this.value = val;
                }
            }
        } catch (ex: any) {
            this.value = undefined;
            this.error = ex;
        }
    }


    protected willUpdate(changed: PropertyValues): void {
        if (changed.has("value")) {
            this.dispatchEvent(new CustomEvent<number | undefined>("change", { detail: this.value }))
        }

        if (changed.has("modbus") || changed.has("address") || changed.has("deviceId") || changed.has("fetchCmd")) {
            this.fetch();
        }
    }

    valueChange = async (evt: Event) => {
        const input = evt.target as HTMLInputElement;
        const parsed = parseInt(input.value);
        if (Number.isInteger(parsed) && parsed >= 0 && parsed <= 254) {

            if (this.modbus && this.setCmd !== undefined) {
                // if (this.setCmd == -1) {
                //     // Special code for DAPC (set level)
                //     await this.modbus.setHoldingRegister(this.deviceId, this.address, parsed);
                // } else {
                //     // Set DTR0 to the value.
                //     await this.modbus.executeCustomDaliCommand(this.deviceId, 0xa300 | parsed);
                //     // Then execute the setter twice, as attribute setters require two calls within close proximity.
                //     await this.modbus.executeCustomDaliCommand(this.deviceId, this.setCmd! | (this.address << 9), true);
                // }
                // Re-fetch the value.
                this.fetch();
            }
        }
    }

    toggleBit = async (evt: Event) => {
        const el = (evt.target as HTMLInputElement)
        const bit = parseInt(el.dataset.bit!);

        if (this.modbus && bit >= 0 && bit < 15) {
            // Check to see if bit is set in group value. 
            // if (this.value && (1 << bit)) {
            //     // Remove it from the group
            //     await this.modbus.executeCustomDaliCommand(this.deviceId, 0x170 | bit | (this.address << 9), true);
            // } else {
            //     // Add it to the group.
            //     await this.modbus.executeCustomDaliCommand(this.deviceId, 0x160 | bit | (this.address << 9), true);
            // }
            this.fetch();
        }
    }

    toggleCoil = async(evt: Event) => {

    }

    render() {
        const disabled = this.disabled || this.error || this.value === undefined;
        switch(this.type) {
            case 'slider':
                return html`<input id="inp" name="level" type="range" min="0" max="254" value=${this.value?.toString()} ?disabled=${disabled} @change=${this.valueChange}/>`;
            case "bits":
                return repeat(range(8), (idx) => {
                    const bv = Boolean((this.value ?? 0) & (1 << idx));
                    return html`<input type="checkbox" data-bit=${idx+this.setCmd!} @change=${this.toggleBit} ?disabled=${disabled} ?checked=${bv} />`;
                });

            case "coil":
                const isOn = Boolean(this.value && this.value > 0);
                return html`<button type="button" class=${classMap({ btn: true, "btn-lg": true, active: (this.value ?? 0) > 0 })} ?disabled=${this.value === undefined} data-bs-toggle="button" @click=${this.toggleCoil}><i class=${classMap({bi: true, "bi-lightbulb": !isOn, "bi-lightbulb-fill": isOn})}></i></button>`;
        } 
    }


}


declare global {
    interface HTMLElementTagNameMap {
        "modbus-parameter-editor": ModbusParameterEditor;
    }
}
