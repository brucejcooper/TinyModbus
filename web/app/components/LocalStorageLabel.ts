import { html, LitElement, PropertyValues } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import "./ModbusParameterEditor"

export interface ExpandedDetail {
    address: number;
    expanded: boolean;
}

type KeyChangeListener = (key: string) => void;



@customElement("localstorage-label")
export class LocalStorageLabel extends LitElement {

    @property()
    key?: string;

    @state()
    value: string | null = null;

    @property()
    placeholder?: string;

    @property({type: Boolean}) 
    editable = false;

    @property() 
    label: string = "Value";



    static broadcastListeners: KeyChangeListener[] = [];

    static addBroadcastListener(listener: KeyChangeListener) {
        LocalStorageLabel.broadcastListeners.push(listener);
    }

    static removeBroadcastListener(listener: KeyChangeListener) {
        const idx = LocalStorageLabel.broadcastListeners.indexOf(listener);
        if (idx !== -1) {
            LocalStorageLabel.broadcastListeners.slice(idx, 1);
        }
    }

    static broadcastKeyChange(key: string) {
        for (const l of LocalStorageLabel.broadcastListeners) {
            l(key);
        }
    }




    protected createRenderRoot(): HTMLElement | DocumentFragment {
        return this;
    }


    connectedCallback(): void {
        super.connectedCallback();
        LocalStorageLabel.addBroadcastListener(this.keyChanged);
        this.keyChanged();
    }

    disconnectedCallback(): void {
        super.disconnectedCallback();
        LocalStorageLabel.removeBroadcastListener(this.keyChanged);
    }

    protected willUpdate(changed: PropertyValues): void {
        super.willUpdate(changed);
        if (changed.has("key")) {
            this.keyChanged();
        }
    }

    keyChanged = (key?: string) => {
        if (!key || key === this.key) {
            this.value = this.key ? window.localStorage.getItem(this.key) : null;
        }
    }

    changed = (evt: FormDataEvent) => {
        if (this.key) {
            this.value = (evt.target as HTMLInputElement).value
            localStorage.setItem(this.key, this.value);
            LocalStorageLabel.broadcastKeyChange(this.key);
        }
    }

    render() {
        if (this.editable) {
            return html`<div class="input-group mb-3">
                <div class="form-floating">
                    <input class="form-control" id="editor" placeholder=${this.placeholder ?? "No Value"} .value=${this.value} @change=${this.changed}>
                    <label for="editor">${this.label}</label>
                </div>
            </div>`;
        }
        return html`<span>${this.value ?? this.placeholder}</span>`;
    }
}

declare global {
    interface HTMLElementTagNameMap {
        "localstorage-label": LocalStorageLabel;
    }
}
