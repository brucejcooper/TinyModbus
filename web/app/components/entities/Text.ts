import { html, css, LitElement, PropertyValues } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { when } from "lit/directives/when.js";
import { repeat } from "lit/directives/repeat.js";
import { styleMap } from "lit/directives/style-map.js";
import { consume } from "@lit/context";
import { serialContext } from "../SerialContext";
import { chevronDown, chevronRight, InputCss } from "../shared_styles";
import { range } from "lit/directives/range.js";
import { query } from "lit/decorators.js";
import { Entity, SerialConnection } from "../SerialConnection";
import { classMap } from "lit/directives/class-map.js";
import { BaseEntityView } from "./Base";


@customElement("entity-text")
export class TextEntityView extends BaseEntityView {

    @state()
    dragTarget: boolean = false;


    clearValue = (evt: MouseEvent) => {
        this.setValue("none");
        evt.stopPropagation();   
    }

    valueChanged = (evt:Event) => {
        const target = evt.target as HTMLInputElement;
        this.setValue(target.value);
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
    drop = (evt: DragEvent) => {
        this.dragTarget = false;
        const data = evt.dataTransfer?.getData("ha/entityid");
        if (data) {
            this.setValue(data);
        }
    }

    render() {
        const entity = this.entity;
        const entityId = entity.id;

        return html`<div class="input-group mb-3" style=${this.dragTarget ? "border: 1px solid blue; border-radius: 3px;": "border: 1px solid transparent;"} @dragover=${this.dragOver} @dragleave=${this.dragLeave} @drop=${this.drop}>
                <div class="form-floating">
                    <input type="text" class="form-control" id="${entityId}" placeholder="None" .value=${this.entity.value} @change=${this.valueChanged}>
                    <label for=${entityId}>${entity.name ?? entity.id}</label>
                </div>
                <button class="input-group-text" data-entity-id=${entityId} @click=${this.clearValue}><i class="bi bi-x"></i></button>
            </div>`;
    }
}

declare global {
    interface HTMLElementTagNameMap {
        "entity-text": TextEntityView;
    }
}
