// Stand-in for @tauri-apps/api/webview: drag and drop of photos and folders.
import { addFiles, droppedFiles, isPhoto } from "../files";

type DropEvent = { payload: { type: "drop"; paths: string[] } | { type: "over" | "leave" } };

export function getCurrentWebview() {
  return {
    async onDragDropEvent(cb: (e: DropEvent) => void) {
      window.addEventListener("dragover", (e) => {
        e.preventDefault();
        if (e.dataTransfer) e.dataTransfer.dropEffect = "copy";
      });
      window.addEventListener("drop", async (e) => {
        e.preventDefault();
        if (!e.dataTransfer) return;
        const files = (await droppedFiles(e.dataTransfer)).filter((f) => isPhoto(f.name));
        if (!files.length) return;
        cb({ payload: { type: "drop", paths: await addFiles("photos", files, true) } });
      });
      return () => {};
    },
  };
}
