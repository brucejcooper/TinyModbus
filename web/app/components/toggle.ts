import { css } from "lit";


const toggleCss = css`
            input[type="checkbox"].toggle {
                height: 0;
                width: 0;
                visibility: hidden;
            }

            label.toggle {
                cursor: pointer;
                text-indent: -9999px;
                width: 32px;
                height: 16px;
                background: grey;
                display: block;
                border-radius: 100px;
                position: relative;
            }

            label.toggle:after {
                content: "";
                position: absolute;
                top: 2px;
                left: 2px;
                width: 12px;
                height: 12px;
                background: #fff;
                border-radius: 6px;
                transition: 0.2s;
            }

            input:checked + label.toggle {
                background: #bada55;
            }

            input:checked + label.toggle:after {
                left: calc(100% - 2px);
                transform: translateX(-100%);
            }

            label.toggle:active:after {
                width: 32px;
            }
`