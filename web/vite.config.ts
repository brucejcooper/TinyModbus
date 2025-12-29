import { defineConfig } from "vite";
import ws from 'vite-plugin-ws';
import setupWebSocket from "./src/websocket-server";
import { DatabaseSync } from "node:sqlite"


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

const routeRe = /^device\/(?<device>\d+)\/(?<address>\d+)\/(?<cmd>on|off|toggle)(?<params>\?.*)?$/

export default defineConfig({
    plugins: [
        ws({
            path: '/ws',
            setup: setupWebSocket
        }),
    ],
    server: {
        allowedHosts: ["music", "music.tail0802d.ts.net", "music.local", "music.8bitcloud.com"],
        proxy: {
            '/device': {
                target: 'http://localhost:3001',
                bypass: async function (req, res) {
                    try {
                        if (!req.url) {
                            throw new Error("Illegal url");
                        }
                        console.log("Parsing", req.url);
                        const searchIdx = req.url.indexOf("?");
                        const search = searchIdx == -1 ? null : Object.fromEntries(req.url.substring(searchIdx+1).split("&").map(v => { const i = v.split("="); return i.length == 1 ? [i[0], true] : i} ));
                        const path = (searchIdx == -1 ? req.url.substring(1) : req.url.substring(1,searchIdx)).split("/");

                        console.log("Path is ", path, "search", search)

                        let processed = false;

                        if (path.length == 5) {
                            const [deviceLiteral, deviceStr, addressLiteral, addressStr, action] = path;
                            if (deviceLiteral == 'device' && deviceStr.match(/^\d+$/) && addressLiteral === 'address' && addressStr.match(/^\d+$/) && ["on","off","toggle"].includes(action)) {
                                const device = parseInt(deviceStr);
                                const address = parseInt(addressStr);

                                res?.end(JSON.stringify({ staus: "OK"}))


                            }
                        } 

                        if (!processed) {
                            res!.statusCode = 404;
                            res!.end("Not Found");
                        }



                    } catch (ex: any) {
                        res!.statusCode = 500;
                        res?.end(JSON.stringify({ error: ex.toString() }))
                    }
                    return false;

                }
            },
            '/names': {
                target: 'http://localhost:3001',
                bypass: async function (req, res) {
                    try {
                        res?.setHeader('Content-Type', 'application/json')
                        if (req.method == "GET") {

                            const mappings: Mappings = { "0": [], "1": [], "2": [], "3": [] };

                            for (let row of getNames.all()) {
                                mappings[row.device as number][row.address as number] = {
                                    name: row.name as string,
                                    x: row.x as number,
                                    y: row.y as number,
                                };
                            }
                            res?.end(JSON.stringify(mappings))
                        } else if (req.method == "PUT") {
                            await new Promise<void>((resolve, reject) => {
                                const chunks: any[] = [];
                                req.addListener("data", chunk => chunks.push(chunk));
                                req.addListener("end", () => {
                                    try {
                                        const body: ChangeNameInput = JSON.parse(Buffer.concat(chunks).toString('utf8'));
                                        console.log("Body", body);
                                        if (!body.name || body.x == undefined || body.y === undefined || body.deviceId === undefined || body.address === undefined) {
                                            res!.statusCode = 400;
                                            res?.end(JSON.stringify({ error: "Invalid parameters" }))
                                            reject("Invalid parameters");
                                            return;
                                        }
                                        upsertName.run({
                                            device: body.deviceId,
                                            address: body.address,
                                            name: body.name,
                                            x: body.x,
                                            y: body.y,
                                        });
                                        res!.statusCode = 200;
                                        res?.end(JSON.stringify({ ok: true }));
                                        resolve();
                                    } catch (ex: any) {
                                        res!.statusCode = 500;
                                        res?.end(JSON.stringify({ error: ex.toString() }))
                                        reject(JSON.stringify({ error: ex.toString() }))
                                    }

                                })
                            });
                        }
                    } catch (ex: any) {
                        res!.statusCode = 500;
                        res?.end(JSON.stringify({ error: ex.toString() }))
                    }
                    return false;
                }
            }
        }
    }
});
