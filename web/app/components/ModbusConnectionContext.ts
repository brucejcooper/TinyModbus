import {createContext} from '@lit/context';

import type {ModbusConnection} from './ModbusConnection';


export const modbusContext = createContext<ModbusConnection>('modbusconnection');