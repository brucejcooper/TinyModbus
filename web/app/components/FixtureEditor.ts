import { html, LitElement } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { repeat } from "lit/directives/repeat.js";
import { consume } from "@lit/context";
import "./LocalStorageLabel"
import "./DaliLight"
import "./Fixture"
import { serialContext } from "./SerialContext";
import { SerialConnection } from "./SerialConnection";




@customElement("ha-binding-editor")
export class BindingEditor extends LitElement {

    @consume({ context: serialContext })
    serialConnection!: SerialConnection;
    

    @property({attribute: false})
    address: number =0;

    @state()
    dragTarget: boolean = false;

    @state()
    binding: number=0;

    
    protected createRenderRoot(): HTMLElement | DocumentFragment {
        return this;
    }


    dragOver = (evt: DragEvent) => {
        this.dragTarget = true;
        evt.preventDefault();
        if (evt.dataTransfer) {
            evt.dataTransfer.dropEffect = "link";
        }

    }
    dragLeave = (evt: DragEvent) => {
        this.dragTarget = false;        
    }
    drop = async (evt: DragEvent) => {
        this.dragTarget = false;
        const data = parseInt(evt.dataTransfer?.getData("ha/binding") ?? "0");
        if (data) {
            this.setValue(data);
        }
    }

    protected  setValue = async (val: number) => {
        await this.serialConnection.sendLine(`bindings[${this.address}] = 0x${val.toString(16)}`)
    }


    clearValue = (evt: MouseEvent) => {
        this.setValue(0);
        evt.stopPropagation();   
    }

    valueChanged = (evt:Event) => {
        const target = evt.target as HTMLInputElement;
        // this.setValue(target.value);
    }



    render() {
        return html`
            <div class="input-group mb-3" style=${this.dragTarget ? "border: 1px solid blue; border-radius: 3px;": "border: 1px solid transparent;"} @dragover=${this.dragOver} @dragleave=${this.dragLeave} @drop=${this.drop}>
                <div class="form-floating">
                    <input type="text" class="form-control" id="" ?readonly=${true} placeholder="None" .value=${`${this.binding >> 8}/${this.binding & 0xFF}`} @change=${this.valueChanged}>
                    <label for=${this.address}>${this.address}</label>
                </div>
                <button class="input-group-text" @click=${this.clearValue}><i class="bi bi-x"></i></button>
            </div>
        `;
    }
}




export function renderBindingsEditor(fixture: number, bindings: number[]) {
    return html`
        <h3>Switch Fixture</h3>
        <localstorage-label key=${`ha/fixture${fixture}`} placeholder=${`fixture ${fixture}`} ?editable=${true} label="Name"></localstorage-label>

        ${repeat(bindings, (b,idx)=>idx, (binding,idx) => html`
            <ha-binding-editor .address=${fixture*8+idx} .binding=${binding}></ha-binding-editor>
        `)}
    `

}




declare global {
    interface HTMLElementTagNameMap {
        "ha-binding-editor": BindingEditor;
    }
}


