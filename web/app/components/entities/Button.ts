import { html } from "lit";
import { customElement } from "lit/decorators.js";
import { BaseEntityView } from "./Base";


@customElement("entity-button")
export class LightEntityView extends BaseEntityView {

    clicked= () => {
        this.serial.setEntity(this.entity.id, "PRESS");
    }


    render() {
        const entity = this.entity;
        const disabled = false;
        return  html`
            <button type="button" class="btn btn-secondary" ?disabled=${disabled} @click=${this.clicked}>${entity.name ?? entity.id}</button>
        `;
    }
}

declare global {
    interface HTMLElementTagNameMap {
        "entity-button": LightEntityView;
    }
}
