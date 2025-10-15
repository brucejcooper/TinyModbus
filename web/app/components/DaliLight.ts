import { html, css, LitElement, PropertyValues } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { when } from "lit/directives/when.js";
import { repeat } from "lit/directives/repeat.js";
import { styleMap } from "lit/directives/style-map.js";
import { ModbusConnection } from "./ModbusConnection";
import { consume } from "@lit/context";
import { modbusContext } from "./ModbusConnectionContext";
import { chevronDown, chevronRight, InputCss } from "./shared_styles";
import { range } from "lit/directives/range.js";
import { query } from "lit/decorators.js";

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

    @consume({ context: modbusContext })
    @property({ attribute: false })
    public modbus?: ModbusConnection;

    @property({ type: Number })
    address: number = 0;

    @property({ reflect: false })
    level: number = 255;

    @property({ reflect: false })
    min: number = 0;

    @property({ reflect: false })
    max: number = 254;

    @property({ reflect: false })
    powerOnLevel: number = 254;

    @property({ reflect: false })
    systemFailLevel: number = 254;

    @property({ reflect: false })
    lastActive: number = 254;

    @property({ reflect: false })
    groups: number = 0;

    @property()
    deviceId: number = 1;



    willUpdate(changed: PropertyValues) {
        super.willUpdate(changed);


        if (changed.has("modbus")) {
            // console.log("Modbus changed", this.modbus);

            // setTimeout(this.loadValues, 250);
        }
    }

    // loadValues = async () => {
    //     if (this.modbus && this.level !== 255) {
    //         const [minmax, fade, powerLevels, groups] = await Promise.all([
    //             this.modbus.getHoldingRegisters(this.deviceId, 64 + this.address, 1),
    //             this.modbus.getHoldingRegisters(this.deviceId, 2*64 + this.address, 1),
    //             this.modbus.getHoldingRegisters(this.deviceId, 3*64 + this.address, 1),
    //             this.modbus.getHoldingRegisters(this.deviceId, 4*64 + this.address, 1),
    //         ])
    //         this.min = minmax[0] && 0xff;
    //         this.max = minmax[0] >> 8;
    //         this.powerOnLevel = powerLevels[2] && 0xff;
    //         this.powerOnLevel = powerLevels[2] >> 8;
    //         this.groups = groups[0];
    //     }
    // };

    // protected updated(_changedProperties: PropertyValues): void {
    //     super.updated(_changedProperties);
    //     this.renderCanvas();
    // }

    protected createRenderRoot(): HTMLElement | DocumentFragment {
        return this;
    }

    connectedCallback(): void {
        super.connectedCallback();
        // this.label = localStorage.getItem(`dali${this.address}.label`) ?? `Dali${this.address}`;
        // The main app requests the status for All fixtures.  We simply pick out the bits we want.
    }

    labelChanged = (evt: Event) => {
        // this.label = (evt.target as HTMLInputElement).value;
        // localStorage.setItem(`dali${this.address}.label`, this.label);
    };

    startDrag = (evt: Event) => {
        const dragEvent = evt as DragEvent;
        if (dragEvent.dataTransfer) {
            const content = `dali:${this.address}`;
            console.log("Drag", content);
            dragEvent.dataTransfer.setData("modbus/device-address", content);
            dragEvent.dataTransfer.effectAllowed = "link";
        }
    };

    toggleLight = () => {
        console.log("Toggling - current level is", this.level);
        this.modbus?.setCoil(this.deviceId, this.address, this.level == 0);
    };

    canvasMouseMove = (evt: MouseEvent) => {
        // if (this.canvas) {
        //     const rect = this.canvas.getBoundingClientRect();
        //     const x = evt.clientX - rect.left;
        //     const y = evt.clientY - rect.top;
        //     const val = Math.min(Math.max(0, Math.round((254 * (x - 5)) / (this.canvas.width - 10))), 254);
        //     // console.log(val);
        // }
    };

    canvasClick = (evt: MouseEvent) => {
        if (this.modbus) {
            console.log("Click");
            // const rect = this.canvas.getBoundingClientRect();
            // const x = evt.clientX - rect.left;
            // const y = evt.clientY - rect.top;
            // let val = Math.min(Math.max(0, Math.round((254 * (x - 5)) / (this.canvas.width - 10))), 254);

            // if (y < 16) {
            //     // Its a click on the slider itself.  Treat this as a set power level.
            //     console.log("Value is ", val, "min", this.min, "max", this.max);
            //     if (this.min == 255 || (this.min == this.max && this.min == 254)) {
            //         // Its not fadeable.  Just treat it as a toggle.
                    this.toggleLight();
            //     } else {
            //         if (val < this.min) {
            //             val = 0;
            //         }
            //         console.log("Set power to ", val);
            //         this.modbus.setHoldingRegister(this.deviceId, this.address, val);

            //     }
            // }
        }
    };


    render() {
        return html`<div class="daliwrapper" draggable="true" @dragstart=${this.startDrag} >${this.address}</div>`;
        // return html`
        //     <div class="daliwrapper" draggable="true" @dragstart=${this.startDrag}>
        //         <form>
        //             <div class="firstrow">                        
        //                 <input class="underlined" type="text" value=${this.label} @change=${this.labelChanged} style="flex: 1;" />
        //                 ${expanderButton({ expand: this.expanded, click: this.toggleExpanded })}
        //             </div>
        //             <canvas width="300" height="48" id="slider" @mousedown=${this.canvasMouseDown} @mousemove=${this.canvasMouseMove} @click=${this.canvasClick}></canvas>
        //         </form>
        //     </div>
        // `;
    }
}

declare global {
    interface HTMLElementTagNameMap {
        "dali-light": DaliLight;
    }
}
