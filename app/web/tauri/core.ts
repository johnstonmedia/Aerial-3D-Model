// Stand-in for @tauri-apps/api/core in the web demo.
import { invoke as call } from "../backend";

export function invoke<T>(cmd: string, args?: Record<string, unknown>): Promise<T> {
  return call(cmd, args) as Promise<T>;
}
