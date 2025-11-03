import { html } from "lit";
import { customElement } from "lit/decorators.js";
import { BaseEntityView } from "./Base";
import { live } from "lit/directives/live.js";


@customElement("entity-sensor")
export class SensorEntityView extends BaseEntityView {


    render() {
        const val = this.entity?.value;
        return val !== undefined ?
            html`<span>${val}</span>`
            :
            html`<div class="alert alert-warning" role="alert">No Value</div>`
        ;
    }
}

declare global {
    interface HTMLElementTagNameMap {
        "entity-sensor": SensorEntityView;
    }
}
