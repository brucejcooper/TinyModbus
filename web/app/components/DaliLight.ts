import { html, css, LitElement, PropertyValues } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { when } from "lit/directives/when.js";
import { repeat } from "lit/directives/repeat.js";
import { styleMap } from "lit/directives/style-map.js";
import { consume } from "@lit/context";
import { serialContext } from "./SerialContext";
import { chevronDown, chevronRight, InputCss } from "./shared_styles";
import { range } from "lit/directives/range.js";
import { query } from "lit/decorators.js";
import { SerialConnection } from "./SerialConnection";

function expanderButton({ expand, click }: { expand: boolean; click?: (evt: Event) => void }) {
    return html`<button class="expander" @click=${click}>${expand ? chevronDown : chevronRight}</button>`;
}

interface SliderProps {
    value: number;
    max: number;
    min: number;
    lastActive?: number;
    width?: number;
}

export interface ExpandedDetail {
    address: number;
    expanded: boolean;
}

@customElement("dali-light")
export class DaliLight extends LitElement {

    @consume({ context: serialContext })
    @property({ attribute: false })
    public modbus?: SerialConnection;

    @property({ type: Number })
    address: number = 0;

    @property()
    deviceId: number = 1;


    protected createRenderRoot(): HTMLElement | DocumentFragment {
        return this;
    }


    startDrag = (evt: Event) => {
        const dragEvent = evt as DragEvent;
        if (dragEvent.dataTransfer) {
            const content = `dali:${this.address}`;
            console.log("Drag", content);
            dragEvent.dataTransfer.setData("modbus/device-address", content);
            dragEvent.dataTransfer.effectAllowed = "link";
        }
    };

    render() {
        return html`<div class="daliwrapper" draggable="true" @dragstart=${this.startDrag} >
        ${this.address} 
        <i class="bi bi-lightbulb-off"></i>
        </div>`;
    }
}

declare global {
    interface HTMLElementTagNameMap {
        "dali-light": DaliLight;
    }
}
