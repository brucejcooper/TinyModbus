export interface Deferred<T> {
  promise: Promise<T>;
  resolve: (val: T) => void;
  reject: (reason: any) => void;
}

export function makeDeferred<T>(): Deferred<T> {
  let deferred: Partial<Deferred<T>> = {};
  deferred.promise = new Promise((resolve, reject) => {
    deferred.resolve = resolve;
    deferred.reject = reject;
  });
  return deferred as Deferred<T>;
}
