// Stand-in for @tauri-apps/api/event in the web demo.
import { on } from "../backend";

export type UnlistenFn = () => void;

export async function listen<T>(event: string, cb: (e: { payload: T }) => void): Promise<UnlistenFn> {
  return on(event, cb);
}
