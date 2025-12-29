import { html, LitElement } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { repeat } from "lit/directives/repeat.js";
import { consume } from "@lit/context";
import "./LocalStorageLabel"
import "./DaliLight"
import "./Fixture"
import { serialContext } from "./SerialContext";
import { SerialConnection } from "./SerialConnection";
import { deviceNameMappings  } from "./names";


function bindingName(binding: number) {    
    const dev = binding >> 8;
    const addr = binding & 0xFF;
    console.log("Tostring of ", binding, "is", deviceNameMappings[dev.toString()][addr].name)

    return deviceNameMappings[dev.toString()][addr].name ?? `${dev}/${addr}`
}


@customElement("ha-binding-editor")
export class BindingEditor extends LitElement {

    @consume({ context: serialContext })
    serialConnection!: SerialConnection;
    

    @property({attribute: false})
    address: number =0;

    @state()
    dragTarget: boolean = false;

    @state()
    bindings: number[] = [];

    
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
        const newBinding = parseInt(evt.dataTransfer?.getData("ha/binding") ?? "0");

        const newBindingList = [...this.bindings];
        const idx = newBindingList.indexOf(newBinding);
        if (idx == -1) {
            newBindingList.push(newBinding);
        } else {
            newBindingList.splice(idx, 1);
        }
    
        this.setValue(newBindingList);
    }

    protected  setValue = async (val: number[]) => {
        await this.serialConnection.sendLine(`bindings[${this.address}] = {${val.join(",")}}`)
    }


    clearValue = (evt: MouseEvent) => {
        this.setValue([]);
        evt.stopPropagation();   
    }

    valueChanged = (evt:Event) => {
        const target = evt.target as HTMLInputElement;
        // this.setValue(target.value);
    }



    render() {
        console.log("rendering bindings", this.bindings);
        return html`
            <div class="input-group mb-3" style=${this.dragTarget ? "border: 1px solid blue; border-radius: 3px;": "border: 1px solid transparent;"} @dragover=${this.dragOver} @dragleave=${this.dragLeave} @drop=${this.drop}>
                <div class="form-floating">
                    <input type="text" class="form-control" id="" ?readonly=${true} placeholder="None" .value=${`${this.bindings.map(b => bindingName(b)).join(", ") || "None"}`} @change=${this.valueChanged}>
                    <label for=${this.address}>${this.address}</label>
                </div>
                <button class="input-group-text" @click=${this.clearValue}><i class="bi bi-x"></i></button>
            </div>
        `;
    }
}




export function renderBindingsEditor(fixture: number, bindings: number[][]) {

    console.log(`Fixture, ${fixture} with base address ${fixture*8} has bindings ${bindings}`)

    return html`
        <h3>Switch Fixture</h3>
        <localstorage-label .deviceId=${0} .address =${fixture} placeholder=${`fixture ${fixture}`} ?editable=${true} label="Name"></localstorage-label>

        ${repeat(bindings, (b,idx)=>idx, (binding,idx) => html`
            <ha-binding-editor .address=${fixture*8+idx} .bindings=${binding}></ha-binding-editor>
        `)}
    `;

}




// declare global {
//     interface HTMLElementTagNameMap {
//         "ha-binding-editor": BindingEditor;
//     }
// }


