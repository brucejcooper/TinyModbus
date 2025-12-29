import { html, LitElement, PropertyValues, TemplateResult } from "lit";
import { customElement, property, query, state } from "lit/decorators.js";
import { repeat } from "lit/directives/repeat.js";
import { styleMap } from "lit/directives/style-map.js";
import { classMap } from "lit/directives/class-map.js";
import "./LocalStorageLabel"
import "./DaliLight"
import "./Fixture"
import floorplan from "./images/floorplan.png"
import switchplate from "./images/switchplate.png"
import lightOn from "./images/light-on.svg"
import lightOff from "./images/light-off.svg"
import { deviceNameMappings, updateName } from "./names"


@customElement("ha-device-map")
export class DeviceMapEditor extends LitElement {

    @property({attribute: false})
    address: number =0;

    @state()
    dragTarget: boolean = false;

    @query("canvas")
    canvas!: HTMLCanvasElement;

    @query(".imagestack") 
    holder!: HTMLDivElement;


    @state()
    movingElement: HTMLElement|null=null;
    
    @state()
    moveOffset: [number,number] = [0,0];


    @state()
    zoom: number = 0.5;

    @state()
    offset: [number,number] = [0,0];



    protected createRenderRoot(): HTMLElement | DocumentFragment {
        return this;
    }


    connectedCallback() {
        super.connectedCallback();
    }


    // protected updated(_changedProperties: PropertyValues): void {
    //     const ctx = this.canvas?.getContext("2d");
    //     console.log("Rendering", this.canvas, ctx);
    //     if (ctx) {
    //         ctx.strokeStyle = "black";
    //         ctx.clearRect(0,0,this.canvas.width, this.canvas.height);
    //         ctx.beginPath();
    //         ctx.moveTo(100,100);
    //         ctx.lineTo(200,200);
    //         ctx.stroke();
    //     }

    // }

    clicked = (evt: MouseEvent) => {
        const devEl = (evt.target as HTMLElement).closest(".device-map-device") as HTMLElement|undefined;
        if (devEl) {
            console.log("clicked on ", this.movingElement);

        }
    }

    internaldrag = (evt: MouseEvent) => {
        if (this.movingElement) {
            const elOffset = this.holder.getBoundingClientRect();

            const x = evt.x - elOffset.left - this.moveOffset[0];
            const y = evt.y - elOffset.top - this.moveOffset[1];

            this.movingElement.style.left = `${x}px`;
            this.movingElement.style.top = `${y}px`;
        }

    }

    pressed = (evt: MouseEvent) => {
        const devEl = (evt.target as HTMLElement).closest(".device-map-device") as HTMLElement|undefined;
        if (devEl) {
            evt.preventDefault();
            evt.stopPropagation();
            this.movingElement = devEl;
            this.moveOffset = [evt.offsetX, evt.offsetY];
            console.log("pressed ", this.movingElement, this.moveOffset);

            this.holder.addEventListener("mousemove", this.internaldrag);
        }

    }

    released = (evt: MouseEvent) => {
        if (this.movingElement) {
            this.holder.removeEventListener("mousemove", this.internaldrag);
            evt.preventDefault();
            evt.stopPropagation();

            const { device, address } = this.movingElement.dataset;
            if (device !== undefined && address !== undefined) { 
                const elOffset = this.holder.getBoundingClientRect();
                const x = evt.x - elOffset.left - this.moveOffset[0];
                const y = evt.y - elOffset.top - this.moveOffset[1];

                console.log("released ", this.movingElement, x, y);

                const addressInt = parseInt(address!);

                updateName(parseInt(device!), addressInt, { x, y, name: deviceNameMappings[device][addressInt].name });
            }
            this.movingElement = null;
        }
    }


    render() {

        const images: TemplateResult<1>[] = [];
        
        for (const [dev, metas] of Object.entries(deviceNameMappings)) {
            metas.forEach((m, address) => {

                images.push(html`
                    <div class=${classMap({"device-map-device": true, fixture: dev == "0", light: dev !== "0"})} style=${styleMap({ top: `${m.y}px`, left: `${m.x}px` })} data-device=${dev} data-address=${address}>
                    <img src=${dev == "0" ? switchplate : lightOn } style=${styleMap({ width: "16px" })}>
                    <span>${m.name}</span>
                    </div>
                    `);
            })
        }

        return html`
            <div class="imagestack" @click=${this.clicked} @mousedown=${this.pressed} @mouseup=${this.released}>
                <div style=${styleMap({ transform: `scale(${this.zoom}) translate(${this.offset[0]}px, ${this.offset[1]}px)`, position: "relative"})}>
                    <img src=${floorplan} style="top:0, left: 0"/>
                    ${images}
                </div>
            <div>
        `;
    }
}

