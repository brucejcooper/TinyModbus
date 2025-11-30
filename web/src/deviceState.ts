
export type DaliLight = {
    status: number;
    brightness: number;
    min: number;
    max: number;
    poweron: number;
    powerfail: number;
    faderate: number;
    fadetime: number;
    groups: number;
}

export type PotentiallyDisabled<X> = X|null;

export type DaliGatewayDevice = PotentiallyDisabled<PotentiallyDisabled<DaliLight>[]>
export type ModbusGatewayDevice = PotentiallyDisabled<boolean[]>;

export type GatewayDeviceState = {
    modbus: Record<string,ModbusGatewayDevice>;
    dali: Record<string,DaliGatewayDevice>;
    bindings: number[];
    buttons: boolean[];
};