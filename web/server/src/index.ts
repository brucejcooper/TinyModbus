import Koa from 'koa';
import logger from "koa-logger";
import Router from '@koa/router';
import { koaBody } from "koa-body";
require('dotenv').config()
import { DatabaseSync } from "node:sqlite"
import websockify from 'koa-websocket';

const database = new DatabaseSync("names.db");
database.exec("CREATE TABLE IF NOT EXISTS names ( device INTEGER NOT NULL, address INTEGER NOT NULL, name TEXT, x INTEGER NOT NULL, y INTEGER NOT NULL, PRIMARY KEY (device, address))");
const upsertName = database.prepare(`INSERT INTO names(device, address, name, x, y) VALUES(:device, :address, :name, :x, :y) ON CONFLICT DO UPDATE SET name=:name, x=:x, y=:y`);
const getNames = database.prepare("SELECT * from names")

type DeviceMeta = {
    name: string;
    x: number;
    y: number;
}

type Mappings = Record<string, DeviceMeta[]>;
type ChangeNameInput = {
    deviceId: number;
    address: number;
    name: string;
    x: number;
    y: number;
}


const expectedToken = process.env.EXPECTED_TOKEN;

if (!expectedToken) {
    throw new Error("EXPECTED_TOKEN environment must be specified and non-0-length");
}

const app = websockify(new Koa(), {});
app.use(logger());
app.use(koaBody());


// Authoriser - We only accept requests that have the correct token.
app.use((ctx, next) => {
    if (ctx.query.token !== expectedToken) {
        ctx.status = ctx.query.token ? 403 : 401;
    } else {
        next();
    }
})


const router = new Router();


router.get("/names", async (ctx) => {
    ctx.setHeader('Content-Type', 'application/json')
    const mappings: Mappings = { "0": [], "1": [], "2": [], "3": [] };

    for (let row of getNames.all()) {
        mappings[row.device as number][row.address as number] = {
            name: row.name as string,
            x: row.x as number,
            y: row.y as number,
        };
    }
    ctx.body = mappings;
});


router.put("/names", async (ctx) => {
    const body = ctx.body as ChangeNameInput;
    console.log("Body", body);
    if (!body.name || body.x == undefined || body.y === undefined || body.deviceId === undefined || body.address === undefined) {
        ctx.throw(400, "Invalid Parameters");
    }
    upsertName.run({
        device: body.deviceId,
        address: body.address,
        name: body.name,
        x: body.x,
        y: body.y,
    });
    ctx.body = { ok: true};
});




router.get('/device/:dev/address/:addr/on', async (ctx) => {
    console.log("ON, params", ctx.params, "Search", ctx.query);
    const { dev, addr } = ctx.params;
    console.log(`turning device ${dev} address ${addr} on for ${ctx.query.duration ? `${ctx.query.duration} seconds` : 'indefinitely'}`)
    ctx.status = 202;
    // req.body = "OK";
});

app.use(router.routes());


const port = 3000;
app.listen(port, () => {
    console.log(`Server is running on port ${port}`);
});
