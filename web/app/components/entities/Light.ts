import { html } from "lit";
import { customElement } from "lit/decorators.js";
import { BaseEntityView } from "./Base";
import { live } from "lit/directives/live.js";


@customElement("entity-light")
export class LightEntityView extends BaseEntityView {

    onOffChange= (evt: InputEvent) => {
        const inp = evt.target as HTMLInputElement
        this.serial.setEntity(this.entity.id, { state: inp.checked ? "ON": "OFF"});
    }

    brightnessChange = (evt: InputEvent) => {
        const inp = evt.target as HTMLInputElement
        this.serial.setEntity(this.entity.id, { brightness: parseInt(inp.value)});
    }

    render() {
        const entity = this.entity;
        const value = entity.value;
        const brightness = value == undefined ? 0 : value.brightness ?? value.state === 'ON' ? 254 : 0;
        const disabled = false;

        return value?.brightness !== undefined ? html`
            <input id="inp" name="level" type="range" min="0" max="254" .value=${live(brightness?.toString())} ?disabled=${disabled} @change=${this.brightnessChange}/>
        `
            :
            html`<input id="inp" name="on/off" type="checkbox" ?checked=${value.state == "ON"} ?disabled=${disabled} @change=${this.onOffChange}/>`
        ;
    }
}

declare global {
    interface HTMLElementTagNameMap {
        "entity-light": LightEntityView;
    }
}
