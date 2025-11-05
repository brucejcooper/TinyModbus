import { html } from "lit";
import { customElement } from "lit/decorators.js";
import { BaseEntityView } from "./Base";
import { live } from "lit/directives/live.js";


@customElement("entity-number")
export class NumberEntityView extends BaseEntityView {

    valueChange = (evt: InputEvent) => {
        const inp = evt.target as HTMLInputElement
        this.serial.setEntity(this.entity.id, parseInt(inp.value));
    }

    render() {
        const entity = this.entity;
        const value = entity.value;
        const disabled = false;

        return html`
            <input id="inp" name="level" type="range" min="0" max="254" .value=${live(value?.toString())} ?disabled=${disabled} @change=${this.valueChange}/>
        `
        ;
    }
}

declare global {
    interface HTMLElementTagNameMap {
        "entity-number": NumberEntityView;
    }
}
