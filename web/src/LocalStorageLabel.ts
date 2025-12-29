import { html, LitElement, PropertyValues } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { addNameChangeListener, DeviceMeta, deviceNameMappings, removeNameChangeListener, updateName} from "./names";

export interface ExpandedDetail {
    address: number;
    expanded: boolean;
}


function afterUnderscore(val: string|undefined){
    if (!val)
        return val;
    const pos = val.lastIndexOf("_");
    return val.substring(pos+1);
}

@customElement("localstorage-label")
export class LocalStorageLabel extends LitElement {

    @property({type: Number})
    deviceId: number = 0;

    @property({type: Number})
    address: number = 0;


    @state()
    meta: DeviceMeta | null = null;

    @property()
    placeholder?: string;

    @property({type: Boolean}) 
    editable = false;

    @property() 
    label: string = "Value";








    protected createRenderRoot(): HTMLElement | DocumentFragment {
        return this;
    }


    connectedCallback(): void {
        super.connectedCallback();
        addNameChangeListener(this.reactToChange);
        this.reactToChange(this.deviceId, this.address);
    }

    disconnectedCallback(): void {
        super.disconnectedCallback();
        removeNameChangeListener(this.reactToChange);
    }

    protected willUpdate(changed: PropertyValues): void {
        super.willUpdate(changed);
        if (changed.has("address") || changed.has("deviceId")) {
            this.reactToChange(this.deviceId, this.address);
        }
    }

    reactToChange = async (deviceId: number, address: number) => {
        console.log("Local name listener received change of ", deviceId, address);
        if (this.deviceId == deviceId && this.address == address) {
            this.meta = deviceNameMappings[this.deviceId][this.address];
            console.log("New Meta ", this.meta);
            this.requestUpdate()
        }
    }

    changed = async (evt: FormDataEvent) => {
        if (this.meta) {
            this.meta.name = (evt.target as HTMLInputElement).value
            await updateName(this.deviceId, this.address, this.meta);
        }
    }

    render() {

        const placeholder = this.placeholder ?? `Dev ${this.deviceId} Address ${this.address}`;

        if (this.editable) {
            return html`<div class="input-group mb-3">
                <div class="form-floating">
                    <input class="form-control" id="editor" placeholder=${placeholder ?? "No Value"} .value=${this.meta?.name ?? placeholder} @change=${this.changed}>
                    <label for="editor">${this.label}</label>
                </div>
            </div>`;
        }
        return html`<span>${this.meta?.name ?? placeholder}</span>`;
    }
}

// declare global {
//     interface HTMLElementTagNameMap {
//         "localstorage-label": LocalStorageLabel;
//     }
// }
