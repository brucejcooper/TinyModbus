import {createContext} from '@lit/context';



export const selectedDeviceContext = createContext<string|null>('selectedDevice');