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


@customElement("entity-binary-sensor")
export class BinarySensorEntityView extends BaseEntityView {
    render() {
        const entity = this.entity;
        const isOn = entity.value === "ON";
        console.log("Entity", this.id, "Value is", isOn);
        return html`<div>
            <i class=${classMap({ bi: true, "bi-toggle-off": !isOn, "bi-toggle-on": isOn})}></i>
        </div>`;
    }
}

declare global {
    interface HTMLElementTagNameMap {
        "entity-binary-sensor": BinarySensorEntityView;
    }
}
