
export interface Serial {
  getPorts(): Promise<SerialPort[]>;
  requestPort(): Promise<SerialPort>;
  addEventListener(evt: "connect" | "disconnect", handler: (evt: Event) => any): void;
}

export interface SerialSignals {
  clearToSend: boolean;
  dataCarrierDetect: boolean;
  dataSetReady: boolean;
  ringIndicator: boolean;
}

export interface SerialPort {
  open(options: {
    baudRate: number;
    bufferSize?: number;
    dataBits?: number;
    flowControl?: "none" | "hardware";
    parity?: "none" | "even" | "odd";
    stopBits?: number;
  }): Promise<void>;
  close(): Promise<void>;
  forget(): Promise<undefined>;
  getInfo(): { usbVendorId?: string; usbProductId?: string; bluetoothServiceClassId?: number | string };
  getSignals(): Promise<SerialSignals>;
  setSignals(signals: { dataTerminalReady?: boolean; requestToSend?: boolean; break?: boolean }): Promise<void>;

  readable?: ReadableStream;
  writable?: WritableStream;
  connected: boolean;
}
