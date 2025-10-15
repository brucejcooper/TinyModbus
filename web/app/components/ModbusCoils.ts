import { html, css, LitElement } from "lit";
import { customElement, property } from "lit/decorators.js";
import { range } from "lit/directives/range.js";
import { map } from "lit/directives/map.js";
import { classMap } from "lit/directives/class-map.js";
import { ModbusConnection } from "./ModbusConnection";
import { consume } from "@lit/context";
import { modbusContext } from "./ModbusConnectionContext";

@customElement("modbus-coils")
export class ModbusCoils extends LitElement {
  static styles = css`
    :host {
      display: flex;
      flex-direction: row;
      gap: 0.33em;
      flex-wrap: wrap;
    }

    button {
      border: 1px solid var(--bs-btn-border-color);
      border-radius: 0.375rem;
      padding: 0.375rem 0.5rem;
      min-width: 2.5em;
      color: white;
      font-size: 1rem;
    }

    button:hover {
      background-color: #0b5ed7;
    }

    .coil-on {
      background-color: var(--bs-primary);
    }
    .coil-off {
      background-color: var(--bs-secondary);
    }

    button:hover.coil-off {
      background-color: #565e64
    }
  `;

  @consume({ context: modbusContext })
  @property({ attribute: false })
  public modbus?: ModbusConnection;

  @property({ reflect: false })
  relays: number = 0;

  coilFetcherTimer?: number;

  private  fetchCoils = async () => {
    if (this.modbus?.port?.connected) {
      const coils = await this.modbus?.getCoils(1, 0, 32);;
      const newCoils = Number(coils);
      if (newCoils !== this.relays) {
        this.relays = newCoils;
      }
    }
  };

  connectedCallback(): void {
    super.connectedCallback();
    this.coilFetcherTimer = setInterval(this.fetchCoils, 50) as unknown as number;
  }

  disconnectedCallback(): void {
    super.disconnectedCallback();
    if (this.coilFetcherTimer) {
      clearInterval(this.coilFetcherTimer);
    }
  }

  coilClicked(evt: Event) {
    const id = parseInt((evt.target as HTMLButtonElement).innerText);
    const current = (this.relays & (1 << id)) !== 0;
    this.modbus?.setCoil(1, id, !current);
  }

  startDrag = (evt: Event) => {
    const dragEvent = (evt as DragEvent);
    const coilId = (dragEvent.target as HTMLElement).textContent?.trim();
    if (coilId && dragEvent.dataTransfer) {
      const content = `modbus:${coilId}`;
      dragEvent.dataTransfer.setData("modbus/device-address", content);
      dragEvent.dataTransfer.effectAllowed = "link";
  
    }
  }

  render() {
    return map(range(32), (idx) => {
      const coil = (this.relays & (1 << idx)) != 0;
      // console.log("coil", idx, coil, this.coils);
      return html`<div class="coil" draggable="true" @dragstart=${this.startDrag}>
        <button
          class=${classMap({"coil-on": coil, "coil-off": !coil })}
          @click=${this.coilClicked}
        >
          ${idx}
        </button>
      </div>`;
    });
  }
}

declare global {
  interface HTMLElementTagNameMap {
    "modbus-coils": ModbusCoils;
  }
}
