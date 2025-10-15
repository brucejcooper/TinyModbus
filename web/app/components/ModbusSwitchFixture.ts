import { html, css, LitElement, PropertyValues } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { range } from "lit/directives/range.js";
import { map } from "lit/directives/map.js";
import { when } from "lit/directives/when.js";
import { classMap } from "lit/directives/class-map.js";
import { ModbusCommand, ModbusConnection } from "./ModbusConnection";
import { consume } from "@lit/context";
import { modbusContext } from "./ModbusConnectionContext";
import { InputCss } from "./shared_styles";

export interface SwitchFixtureID {
    fixture: number;
    button: number;
}
interface ModbusButtonBinding {
    modbus: { address: number };
}
interface DaliButtonBinding {
    dali: { address: number };
}

type ButtonBinding = null | ["dali" | "modbus", number];

function decodeBinding(bindingVal: number): ButtonBinding {
    const type = bindingVal >> 14;
    if (type == 0) {
        return ["modbus", bindingVal & 0x3fff];
    } else if (type == 1) {
        return ["dali", bindingVal & 0x3fff];
    }
    return null;
    // throw new Error("Illgal binding");
}

function encodeBinding(binding: ButtonBinding): number {
    if (!binding) {
        return 0xc000;
    }
    switch (binding[0]) {
        case "dali":
            return (1 << 14) | (binding[1] & 0x3fff);
        case "modbus":
            return binding[1] & 0x3fff;
        default:
            return 3 << 14;
    }
}

@customElement("modbus-switch-fixture")
export class ModbusSwitchFixture extends LitElement {
    static styles = [
        css`
            .switchFixture {
                border: 1px solid #777;
                border-radius: 16px;
                display: flex;
                gap: 5px;
                min-width: 35px;
                /* align-items: center; */
                flex-direction: column;
                padding: 8px;
            }

            .switchFixture .switch {
                width: 16px;
                height: 16px;
                border-radius: 6px;
                border: 1px solid #999;
                margin-right: 0.25em;
            }

            .switchfixture .switch_on {
                background-color: blue;
            }
            .binding {
                font-size: 80%;
            }

            .binding img {
                width: 16px;
            }

            .wrapper {
                display: flex;
                flex-direction: column;
                align-items: center;
                gap: 0.25em;
            }

            .stateandbinding {
                display: flex;
                flex-direction: row;
                align-items: center;
                border: 1px solid transparent;
            }

            .stateandbinding.dropTarget {
                border-color: red;
            }

            button.delbinding {
                border: none;
                background: none;
                cursor: pointer;
                color: var(--bs-danger);
                font-family: -apple-system, BlinkMacSystemFont, Segoe UI, Roboto, Oxygen-Sans, Ubuntu, Cantarell, Helvetica Neue, sans-serif;
                font-size: 110%;
            }

            button.delbinding:hover {
                color: var(--bs-orange);
            }
        `,
        InputCss,
    ];

    @consume({ context: modbusContext })
    @property({ attribute: false })
    public modbus?: ModbusConnection;

    @property({ type: Number })
    fixture: number = 0;

    @property({ reflect: false })
    buttons: number = 0;

    @property({ reflect: false })
    label: string = "";

    @property({ reflect: false })
    bindings: ButtonBinding[] = [];

    @state()
    dropTargetButton: number = -1;

    requestBindings = async () => {
        if (this.modbus) {
            this.bindings = (await this.modbus.getHoldingRegisters(1, this.fixture * 7, 7)).map(decodeBinding);
        }
    };

    connectedCallback(): void {
        super.connectedCallback();
        this.label = localStorage.getItem(`modbus.fixture${this.fixture}`) ?? `Fix${this.fixture}`;
        // The main app requests the status for All fixtures.  We simply pick out the bits we want.
        setTimeout(this.requestBindings, 200);
    }

    labelChanged = (evt: Event) => {
        this.label = (evt.target as HTMLInputElement).value;
        localStorage.setItem(`modbus.fixture${this.fixture}`, this.label);
    };

    buttonClicked = (evt: Event) => {
        const target = evt.target as HTMLButtonElement;
        this.dispatchEvent(new CustomEvent<SwitchFixtureID>("switch-selected", { detail: { button: Number(target.dataset.index), fixture: this.fixture }, bubbles: true }));
    };

    protected willUpdate(_changedProperties: PropertyValues): void {
        super.willUpdate(_changedProperties);
    }


    private async changeBindingAtIndex(index: number, value: ButtonBinding) {
        await this.modbus?.setHoldingRegister(1, this.fixture * 7 + this.dropTargetButton, encodeBinding(value));
        const newBindings = [...this.bindings];
        newBindings[index] = value;
        this.bindings = newBindings;
    }

    clearBinding = (evt: Event) => {
        const index = parseInt((evt.target as HTMLButtonElement).dataset.index!);
        this.modbus?.setHoldingRegister(1, this.fixture * 7 + index, 0xc000);
        this.changeBindingAtIndex(index, null);
    };

    drop = (evt: Event) => {
        const data = (evt as DragEvent).dataTransfer?.getData("modbus/device-address");
        if (data) {
            console.log("Binding Fixture", this.fixture, "button ", this.dropTargetButton, "to", data);
            const [bus, addressStr] = data.split(":");
            const address = parseInt(addressStr);
            this.changeBindingAtIndex(this.dropTargetButton, [bus as "dali" | "modbus", address]);
            this.dropTargetButton = -1;
        }
    };

    dragover = (evt: Event) => {
        const ev = evt as DragEvent;
        const tgt = (evt.target as HTMLElement).closest(".stateandbinding");
        if (tgt) {
            ev.preventDefault();
            const idx = (tgt.querySelector(".switch") as HTMLElement | null)?.dataset.index;
            this.dropTargetButton = idx == undefined ? -1 : Number(idx);
            if (ev.dataTransfer) {
                ev.dataTransfer.dropEffect = "link";
            }
        }
    };

    dragleave = (evt: Event) => {
        this.dropTargetButton = -1;
    };

    render() {
        return html`
            <div class="wrapper">
                <input class="underlined" type="text" value=${this.label} @change=${this.labelChanged} />
                <div class="switchFixture">
                    ${map(range(5), (btnidx) => {
                        const binding = this.bindings[btnidx];
                        return html`<div
                            class=${classMap({ stateandbinding: true, dropTarget: btnidx == this.dropTargetButton })}
                            @drop=${this.drop}
                            @dragover=${this.dragover}
                            @dragleave=${this.dragleave}
                        >
                            <button data-index=${btnidx} @click=${this.buttonClicked} class=${classMap({ switch: true, switch_on: (this.buttons & (1 << btnidx)) !== 0 })}></button>
                            ${when(binding, () => html`
                                <span class="binding"> ${binding && `${binding[0] === "modbus" ? "relay" : binding[0]} ${binding[1]}`} </span>
                                <button data-index=${btnidx} class="delbinding" @click=${this.clearBinding}>&#x2717;</button>
                            `)}
                        </div>`;
                    })}
                </div>
            </div>
        `;
    }
}

declare global {
    interface HTMLElementTagNameMap {
        "modbus-switch-fixture": ModbusSwitchFixture;
    }
}
