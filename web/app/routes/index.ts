import { html } from "lit";
import {defaultLayout} from "../layouts/layout.js";


export const indexPage = () => defaultLayout(html`
    <modbus-app></modbus-app>
`);
