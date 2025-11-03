import { html, css, LitElement, PropertyValues } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { consume } from "@lit/context";
import { serialContext } from "./SerialContext";
import { classMap } from "lit/directives/class-map.js";
import "./ModbusParameterEditor"
import { ModbusParameterEditor } from "./ModbusParameterEditor";
import { SerialConnection } from "./SerialConnection";

export interface ExpandedDetail {
    address: number;
    expanded: boolean;
}

@customElement("dali-light-editor")
export class DaliLightEditor extends LitElement {
    @consume({ context: serialContext })
    public modbus!: SerialConnection;

    @property({ type: Number })
    deviceId: number = 1;


    @property({ type: Number })
    address: number = -1;

    @property({ reflect: false })
    label: string = "";

    @state()
    min: number = 254;

    @state()
    max: number = 254;



    protected createRenderRoot(): HTMLElement | DocumentFragment {
        return this;
    }

    connectedCallback(): void {
        super.connectedCallback();
        this.label = localStorage.getItem(`modbus/device${this.deviceId}/addr${this.address}/label`) ?? `Dali${this.address}`;
    }

    labelChanged = (evt: Event) => {
        this.label = (evt.target as HTMLInputElement).value;
        localStorage.setItem(`modbus/device${this.deviceId}/addr${this.address}/label`, this.label);
    };

    private requestLevelUpdate() {
        (this.querySelector("#level_slider") as ModbusParameterEditor)?.fetch();
    }

    toggle = async (evt: MouseEvent) => {
        // await this.modbus?.setCoil(this.deviceId, this.address, "toggle");
        // Dali takes _ages_ to update its values after a command is sent.  
        await new Promise((resolve) => setTimeout(resolve, 100));
        this.requestLevelUpdate();
    }

    minChanged = (evt: CustomEvent<number>) => { this.min = evt.detail;}
    maxChanged = (evt: CustomEvent<number>) => { this.max = evt.detail;}

    disconnectedCallback(): void {
        super.disconnectedCallback();
        this.removeEventHandler();
    }

    polledValuesChanged = () => {
        this.requestUpdate();
    }

    private removeEventHandler() {
        if (this.pollHandlerFilter) {
            // this.modbus.removePollChangeHandler(this.polledValuesChanged, this.pollHandlerFilter);
        }
    }

    private pollHandlerFilter: { deviceId: number; addressRange: [number,number] } | undefined;


    private refreshEventHandler() {
        this.removeEventHandler();
        this.pollHandlerFilter = { deviceId: this.deviceId, addressRange: [this.address, this.address ] };
        // this.modbus.addPollChangeHandler(this.polledValuesChanged, this.pollHandlerFilter);
    }


    protected willUpdate(changed: PropertyValues): void {
        super.willUpdate(changed);

        if (this.modbus && (changed.has("deviceId") || changed.has("type"))) {
            this.refreshEventHandler();
        }
    }


    render() {

        const level = 0; //this.modbus.polledValues.get(this.deviceId)?.holding[this.address];

        const isOn = level !== 0;

        return html`
        <div class="lighteditor">
            <div class="fs-4">DALI Bus ${this.deviceId} Light ${this.address}    
                <button type="button" class=${classMap({ btn: true, "btn-lg": true, active: (level ?? 0) > 0 })} ?disabled=${level === undefined} data-bs-toggle="button" @click=${this.toggle}><i class=${classMap({bi: true, "bi-lightbulb": !isOn, "bi-lightbulb-fill": isOn})}></i></button>
            </div> 

            <localstorage-label key=${`modbus/label/${this.deviceId}/${this.address}`} editable label="Label" placeholder=${this.address}></localstorage-label>

            <form class="details">
                <label for="level_slider" class="form-label">Level</label>
                <modbus-parameter-editor id="level_slider" label="Level" .fetchCmd=${0x1a0} .setCmd=${-1} .deviceId=${this.deviceId} .address=${this.address} ></modbus-parameter-editor>


                <label for="minLevel" class="form-label">Min</label>
                <modbus-parameter-editor id="minLevel" label="Min" .fetchCmd=${0x1a2} .setCmd=${0x12b} .deviceId=${this.deviceId} .address=${this.address} @change=${this.minChanged}></modbus-parameter-editor>

                <label for="maxLevel" class="form-label">Max</label>
                <modbus-parameter-editor id="maxLevel" label="Max" .fetchCmd=${0x1a1} .setCmd=${0x12a} .deviceId=${this.deviceId} .address=${this.address} @change=${this.maxChanged}></modbus-parameter-editor>


                <label for="powerOn" class="form-label">Power on</label>
                <modbus-parameter-editor id="powerOn" label="Power on" .fetchCmd=${0x1a3} .setCmd=${0x12d} .deviceId=${this.deviceId} .address=${this.address}></modbus-parameter-editor>

                <label for="sysFail" class="form-label">System failure</label>
                <modbus-parameter-editor id="sysFail" label="Max" .fetchCmd=${0x1a4} .setCmd=${0x12c} .deviceId=${this.deviceId} .address=${this.address}></modbus-parameter-editor>


                <label for="fade_rate" class="form-label">Fade Rate</label>
                <input name="fade_rate" type="number"  /> 
                <label for="fade_time" class="form-label">Fade Time</label>
                <input name="fade_time" type="number"  />


                <label for="groups0" class="form-label">Groups 0-7</label>
                <modbus-parameter-editor type="bits" id="groups0" label="Groups0-7" .fetchCmd=${0x1c0} .setCmd=${0} .deviceId=${this.deviceId} .address=${this.address}></modbus-parameter-editor>

                <label for="groups8" class="form-label">Groups 8-15</label>
                <modbus-parameter-editor type="bits" id="groups8" label="Groups8-15" .fetchCmd=${0x1c1} .setCmd=${8} .deviceId=${this.deviceId} .address=${this.address}></modbus-parameter-editor>
            </form>
        </div>
        `;
    }
}

declare global {
    interface HTMLElementTagNameMap {
        "dali-light-editor": DaliLightEditor;
    }
}
