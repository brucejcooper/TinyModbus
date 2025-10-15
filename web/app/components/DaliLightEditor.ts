import { html, css, LitElement, PropertyValues } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { when } from "lit/directives/when.js";
import { repeat } from "lit/directives/repeat.js";
import { styleMap } from "lit/directives/style-map.js";
import { ModbusConnection } from "./ModbusConnection";
import { consume } from "@lit/context";
import { modbusContext } from "./ModbusConnectionContext";
import { chevronDown, chevronRight, InputCss } from "./shared_styles";
import { range } from "lit/directives/range.js";
import { query } from "lit/decorators.js";
import { classMap } from "lit/directives/class-map.js";


export interface ExpandedDetail {
    address: number;
    expanded: boolean;
}

@customElement("dali-bound-slider")
export class DaliBoundSlider extends LitElement {

    static styles = css`
    :host {
        display: inline-flex;
    }

    input {
        width: 100%;
    }
    `;

    @consume({ context: modbusContext })
    public modbus?: ModbusConnection;

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
    type: "slider" | "bits" = "slider";

    private abortCtrl?: AbortController;

    fetch = async () => {
        if (this.abortCtrl) {
            this.abortCtrl.abort();
        }
        this.abortCtrl = new AbortController();
        const signal = this.abortCtrl.signal;
        this.error = null;
        try {
            if (this.modbus && this.deviceId > 0 && (this.address !== -1) && this.fetchCmd) {
                const val = await this.modbus.executeCustomDaliCommand(this.deviceId, this.fetchCmd | (this.address << 9));
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
                if (this.setCmd == -1) {
                    // Special code for DAPC (set level)
                    await this.modbus.setHoldingRegister(this.deviceId, this.address, parsed);
                } else {
                    // Set DTR0 to the value.
                    await this.modbus.executeCustomDaliCommand(this.deviceId, 0xa300 | parsed);
                    // Then execute the setter twice, as attribute setters require two calls within close proximity.
                    await this.modbus.executeCustomDaliCommand(this.deviceId, this.setCmd! | (this.address << 9), true);
                }
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
            if (this.value && (1 << bit)) {
                // Remove it from the group
                await this.modbus.executeCustomDaliCommand(this.deviceId, 0x170 | bit | (this.address << 9), true);
            } else {
                // Add it to the group.
                await this.modbus.executeCustomDaliCommand(this.deviceId, 0x160 | bit | (this.address << 9), true);
            }
            this.fetch();
        }
    }

    render() {
        console.log("Render", this.value, this.disabled);
        const disabled = this.disabled || this.error || this.value === undefined;
        switch(this.type) {
            case 'slider':
                return html`<input id="inp" name="level" type="range" min="0" max="254" .value=${this.value} ?disabled=${disabled} @change=${this.valueChange}/>`;
            case "bits":
                return repeat(range(8), (idx) => {
                    const bv = Boolean((this.value ?? 0) & (1 << idx));
                    return html`<input type="checkbox" data-bit=${idx+this.setCmd!} @change=${this.toggleBit} ?disabled=${disabled} ?checked=${bv} />`;
            });
        } 
    }


}



@customElement("dali-light-editor")
export class DaliLightEditor extends LitElement {
    @consume({ context: modbusContext })
    public modbus?: ModbusConnection;

    @property()
    deviceId: number = 1;


    @property({ type: Number })
    address: number = -1;

    @property({ reflect: false })
    label: string = "";

    @state()
    level = 254;

    @state()
    min: number = 254;

    @state()
    max: number = 254;

    fetcherId: number = -1;


    protected createRenderRoot(): HTMLElement | DocumentFragment {
        return this;
    }

    connectedCallback(): void {
        super.connectedCallback();
        this.label = localStorage.getItem(`dali${this.address}.label`) ?? `Dali${this.address}`;
        // The main app requests the status for All fixtures.  We simply pick out the bits we want.

        // this.fetcherId = setInterval(this.loadValues, 500) as unknown as number;
    }

    disconnectedCallback(): void {
        if (this.fetcherId !== -1) {
            clearInterval(this.fetcherId);
        }
    }

    labelChanged = (evt: Event) => {
        this.label = (evt.target as HTMLInputElement).value;
        localStorage.setItem(`dali${this.address}.label`, this.label);
    };

    private requestLevelUpdate() {
        (this.querySelector("#level_slider") as DaliBoundSlider)?.fetch();
    }

    toggle = async () => {
        await this.modbus?.setCoil(this.deviceId, this.address, this.level == 0);
        // Dali takes _ages_ to update its values after a command is sent.  
        await new Promise((resolve) => setTimeout(resolve, 100));
        this.requestLevelUpdate();
    }

    levelChanged = (evt: CustomEvent<number>) => { this.level = evt.detail;}
    minChanged = (evt: CustomEvent<number>) => { this.min = evt.detail;}
    maxChanged = (evt: CustomEvent<number>) => { this.max = evt.detail;}


    render() {

        return html`
        <div class="lighteditor">
            <div class="fs-4">DALI Bus ${this.deviceId} Light ${this.address}</div> 

            <button type="button" class=${classMap({ btn: true, active: this.level > 0 })} data-bs-toggle="button" @click=${this.toggle}>${this.level == 0 ? "Off" : "On"}</button>
            <form class="details">

                <label for="level_slider" class="form-label">Level</label>
                <dali-bound-slider id="level_slider" label="Level" .fetchCmd=${0x1a0} .setCmd=${-1} .deviceId=${this.deviceId} .address=${this.address} @change=${this.levelChanged}></dali-bound-slider>


                <label for="minLevel" class="form-label">Min</label>
                <dali-bound-slider id="minLevel" label="Min" .fetchCmd=${0x1a2} .setCmd=${0x12b} .deviceId=${this.deviceId} .address=${this.address} @change=${this.minChanged}></dali-bound-slider>

                <label for="maxLevel" class="form-label">Max</label>
                <dali-bound-slider id="maxLevel" label="Max" .fetchCmd=${0x1a1} .setCmd=${0x12a} .deviceId=${this.deviceId} .address=${this.address} @change=${this.maxChanged}></dali-bound-slider>


                <label for="powerOn" class="form-label">Power on</label>
                <dali-bound-slider id="powerOn" label="Power on" .fetchCmd=${0x1a3} .setCmd=${0x12d} .deviceId=${this.deviceId} .address=${this.address}></dali-bound-slider>

                <label for="sysFail" class="form-label">System failure</label>
                <dali-bound-slider id="sysFail" label="Max" .fetchCmd=${0x1a4} .setCmd=${0x12c} .deviceId=${this.deviceId} .address=${this.address}></dali-bound-slider>


                <label for="fade_rate" class="form-label">Fade Rate</label>
                <input name="fade_rate" type="number"  /> 
                <label for="fade_time" class="form-label">Fade Time</label>
                <input name="fade_time" type="number"  />


                <label for="groups0" class="form-label">Groups 0-7</label>
                <dali-bound-slider type="bits" id="groups0" label="Groups0-7" .fetchCmd=${0x1c0} .setCmd=${0} .deviceId=${this.deviceId} .address=${this.address}></dali-bound-slider>

                <label for="groups8" class="form-label">Groups 8-15</label>
                <dali-bound-slider type="bits" id="groups8" label="Groups8-15" .fetchCmd=${0x1c1} .setCmd=${8} .deviceId=${this.deviceId} .address=${this.address}></dali-bound-slider>
            </form>
        </div>
        `;
    }
}

declare global {
    interface HTMLElementTagNameMap {
        "dali-light-editor": DaliLightEditor;
        "dali-bound-slider": DaliBoundSlider;
    }
}
