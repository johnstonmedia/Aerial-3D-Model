// Turning the user's chosen or dropped files into paths the engine can open.
// Files are registered as handles on the originals — nothing is copied.
import { ingest } from "./backend";

const PHOTO = /\.(jpe?g|png|tiff?)$/i;
let batch = 0;

/** Registers files under /w/<area>/<batch>/… and returns their paths. */
export async function addFiles(area: string, files: File[], keepTree = false): Promise<string[]> {
  const n = ++batch;
  const items = files.map((file) => {
    const rel = keepTree && (file as any).webkitRelativePath ? (file as any).webkitRelativePath : file.name;
    return { path: `/w/${area}/${n}/${rel}`.replace(/\/+/g, "/"), file };
  });
  await ingest(items);
  return items.map((i) => i.path);
}

export function isPhoto(name: string) {
  return PHOTO.test(name);
}

/** Everything inside dropped items, folders walked recursively. */
export async function droppedFiles(dt: DataTransfer): Promise<File[]> {
  const out: File[] = [];
  const walk = async (entry: any, prefix: string): Promise<void> => {
    if (!entry) return;
    if (entry.isFile) {
      const f: File = await new Promise((res, rej) => entry.file(res, rej));
      Object.defineProperty(f, "webkitRelativePath", { value: prefix + f.name });
      out.push(f);
    } else if (entry.isDirectory) {
      const reader = entry.createReader();
      for (;;) {
        const batch: any[] = await new Promise((res, rej) => reader.readEntries(res, rej));
        if (!batch.length) break;
        for (const e of batch) await walk(e, `${prefix}${entry.name}/`);
      }
    }
  };
  const entries = [...dt.items].map((i) => i.webkitGetAsEntry?.()).filter(Boolean);
  if (entries.length) {
    for (const e of entries) await walk(e, "");
  } else {
    out.push(...dt.files);
  }
  return out;
}
