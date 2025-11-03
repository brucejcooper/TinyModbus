import {createContext} from '@lit/context';

import { SerialConnection } from './SerialConnection';


export const serialContext = createContext<SerialConnection>('serialconnection');