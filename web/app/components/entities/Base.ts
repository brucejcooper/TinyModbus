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


@customElement("entity-base")
export abstract class BaseEntityView extends LitElement {

    @consume({ context: serialContext })
    @property({ attribute: false })
    public serial!: SerialConnection;

    @property()
    entity!: Entity;

    @state()
    dragTarget: boolean = false;


    protected createRenderRoot(): HTMLElement | DocumentFragment {
        return this;
    }


    entityChanged = (ent: Entity) => {
        if (ent == this.entity) {
            this.requestUpdate();
        }
    }

    protected setValue = (val: any) => {
        this.serial.setEntity(this.entity.id, val);
    }

    connectedCallback() {
        super.connectedCallback();
        this.entity.device.addEntityChangeListener(this.entityChanged)

    }

    disconnectedCallback() {
        super.disconnectedCallback();
        this.entity.device.removeEntityChangeListener(this.entityChanged)
    }

    render() {
        return html`<div>Please override the render method</div>`;
    }
}

declare global {
    interface HTMLElementTagNameMap {
        "entity-base": BaseEntityView;
    }
}
