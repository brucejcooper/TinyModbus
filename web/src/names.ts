export type DeviceMeta = {
    name: string;
    x: number;
    y: number;
}

export type NameMappings = Record<string, DeviceMeta[]>;


export let deviceNameMappings: NameMappings = {};

type KeyChangeListener = (device: number, address: number) => void;

const broadcastListeners: KeyChangeListener[] = [];



export function addNameChangeListener(listener: KeyChangeListener) {
    broadcastListeners.push(listener);
}

export function removeNameChangeListener(listener: KeyChangeListener) {
    const idx = broadcastListeners.indexOf(listener);
    if (idx !== -1) {
        broadcastListeners.slice(idx, 1);
    }
}

function broadcastKeyChange(deviceId: number, address: number) {
    for (const l of broadcastListeners) {
        l(deviceId, address);
    }
}



export async function loadMappings() {
    const res = await fetch("/names");
    if (!res.ok) {
        throw new Error(`Error calling names (${res.status}): ${await res.text()}`)
    }
    deviceNameMappings = await res.json();
    return deviceNameMappings;
}

export async function updateName(deviceId: number, address: number, meta: DeviceMeta) {
    deviceNameMappings[deviceId][address] = meta;
    const res = await fetch("/names", {
        method: "PUT",
        body: JSON.stringify({
            ...meta,
            deviceId,
            address,
        })
    });

    if (!res.ok) {
        throw new Error(`Error calling names (${res.status}): ${await res.text()}`)
    }

    broadcastKeyChange(deviceId, address);

}


// export async function convertBindings() {
//     const lightNames: Record<string,string[]>= {
//         "0": [],
//     }

//     for (let i = 0; i < 24; i++) {
//         lightNames["0"][i] = localStorage.getItem(`ha/fixture${i}`) ?? `fixture ${i}`;
//     }
//     for (let d = 1; d <= 3; d++) {
//         const deviceLightNames: string[] = [];
//         lightNames[d.toString()] = deviceLightNames;
//         for (let l = 0; l < (d == 1 ? 32 : 64); l++) {
//             deviceLightNames[l] = localStorage.getItem(`ha/${d}/light${l}`) ?? l.toString();
//         }
//     }

    
// }

