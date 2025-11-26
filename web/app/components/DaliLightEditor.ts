import { html, LitElement, TemplateResult } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { consume } from "@lit/context";
import "./LocalStorageLabel"
import "./DaliLight"
import "./Fixture"
import { DaliLight } from "../deviceState";
import { live } from "lit/directives/live.js";
import { serialContext } from "./SerialContext";
import { SerialConnection } from "./SerialConnection";
import { range } from "lit/directives/range.js";
import { repeat } from "lit/directives/repeat.js";
import { when } from "lit/directives/when.js";

interface UpdateDetail {
    attribute: string;
    value: number;
}

const attributeEditorType: Record<string,string> = {
    "faderate": "number",
    "fadetime": "number",
}


@customElement("ha-group-editor")
export class GroupEditor extends LitElement {
    @property({attribute: false})
    light!: DaliLight;


    @property({attribute: false})
    value: number = 0;

        
    protected createRenderRoot(): HTMLElement | DocumentFragment {
        return this;
    }


    valueChange(evt: InputEvent) {
        const input = (evt.target as HTMLInputElement);
        const bit = parseInt(input.dataset.bit!);

        let newVal = this.value ^ (1 << bit);
        console.log("New Group value", newVal.toString(16));

        this.dispatchEvent(new CustomEvent<UpdateDetail>("attribute-change", { bubbles: true, detail: { attribute: "groups", value: newVal }}))
    }

    render() {
        const disabled = false;
        return html`
            ${repeat(range(16), (bit) => bit, (bit) => html`
                ${when(bit == 8, () => html`<br/>`)}
                <input id="inp" name="level" type="checkbox" ?checked=${(this.value >> bit) & 0x01} data-bit=${bit} ?disabled=${disabled} @change=${this.valueChange}/>
            `)}
        `;
    }
}



@customElement("ha-light-status")
export class LightStatus extends LitElement {
    @property({attribute: false})
    light!: DaliLight;

    static bits = ["error", "failure", "power on", "at limit", "fading", "reset", "missingshort", "pwrfail"]


    @property({attribute: false})
    value: number = 0;


        
    protected createRenderRoot(): HTMLElement | DocumentFragment {
        return this;
    }

    render() {
        const status: string[] = [];

        for (let bit = 0; bit < 8; bit++) {
            if (this.value & (1 << bit)) {
                status.push(LightStatus.bits[bit])
            }

        }
        if (status.length == 0) {
            return html`<span>None</span>`;
        }
        return html`<span>${status.join(", ")}</span>`;
    }
}





@customElement("ha-attribute-editor")
export class NumberEditor extends LitElement {
    @property()
    attribute!: string;

    @property({attribute: false})
    light!: DaliLight;


    @property({attribute: false})
    value: number = 0;

        
    protected createRenderRoot(): HTMLElement | DocumentFragment {
        return this;
    }


    valueChange(evt: InputEvent) {
        const input = (evt.target as HTMLInputElement);
        this.dispatchEvent(new CustomEvent<UpdateDetail>("attribute-change", { bubbles: true, detail: { attribute: this.attribute, value: parseInt(input.value)}}))
    }

    render() {
        const disabled = false;
        const editorType = attributeEditorType[this.attribute] ?? 'range';
        const val = this.value?.toString();
        return html`
            <span>
                <input id="inp" name="level" type=${editorType} min="0" max="254" .value=${live(val)} ?disabled=${disabled} @change=${this.valueChange}/>
                ${when(editorType == "range", () => html`<span>${val}</span>`)}
            </span>

        `
    }
}


@customElement("ha-dali-light-editor")
export class DaliLightEditor extends LitElement {

    @consume({ context: serialContext })
    serialConnection!: SerialConnection;

    
    @property({attribute: false})
    light!: DaliLight;

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
        const light = this.light;
        return html`
            <h3>Dali light ${this.deviceid}/${this.address}</h3>

            <localstorage-label key=${`ha/${this.deviceid}/light${this.address}`} label="label" ?editable=${true} placeholder=${`${this.address}`}></localstorage-label>

            <div class="keyvalgrid" @attribute-change=${this.valueChanged}>
                <span>Status</span>     <ha-light-status .light=${light} .value=${light.status}></ha-light-status>
                <span>Brightness</span> <ha-attribute-editor .light=${light} attribute="brightness" .value=${light.brightness}></ha-attribute-editor>
                <span>Min</span>        <ha-attribute-editor .light=${light} attribute="min"        .value=${light.min}></ha-attribute-editor>
                <span>Max</span>        <ha-attribute-editor .light=${light} attribute="max"        .value=${light.max}></ha-attribute-editor>
                <span>Power On</span>   <ha-attribute-editor .light=${light} attribute="poweron"    .value=${light.poweron}></ha-attribute-editor>
                <span>Power Fail</span> <ha-attribute-editor .light=${light} attribute="powerfail"  .value=${light.powerfail}></ha-attribute-editor>
                <span>Fade Rate</span>  <ha-attribute-editor .light=${light} attribute="faderate"   .value=${light.faderate}></ha-attribute-editor>
                <span>Fade Time</span>  <ha-attribute-editor .light=${light} attribute="fadetime"   .value=${light.fadetime}></ha-attribute-editor>
                <span>Groups</span>     <ha-group-editor .light=${light}                            .value=${light.groups}></ha-group-editor>
            </div>
        `;
    }
}





declare global {
    interface HTMLElementTagNameMap {
        "ha-dali-light-editor": DaliLightEditor;
        "ha-group-editor": GroupEditor; 
        "ha-light-status": LightStatus;
    }
}
