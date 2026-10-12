// Stand-in for @tauri-apps/plugin-dialog in the web demo.
//
// open(): a browser file picker; the chosen files become paths in the
//   demo's file system (handles on the originals, not copies). Folders
//   that the desktop app would ask for — a project folder, an output
//   folder — are virtual here, because the browser keeps everything.
// save(): returns a path in the demo's file system; once a command has
//   written it, the file is downloaded.
import { requestDownload } from "../backend";
import { addFiles, isPhoto } from "../files";

interface Filter {
  name: string;
  extensions: string[];
}
interface OpenOptions {
  directory?: boolean;
  multiple?: boolean;
  title?: string;
  filters?: Filter[];
}
interface SaveOptions {
  title?: string;
  defaultPath?: string;
  filters?: Filter[];
}

let outputs = 0;

function pick(opt: { directory?: boolean; multiple?: boolean; accept?: string }): Promise<File[] | null> {
  return new Promise((resolve) => {
    const input = document.createElement("input");
    input.type = "file";
    input.multiple = !!opt.multiple;
    if (opt.directory) (input as any).webkitdirectory = true;
    if (opt.accept) input.accept = opt.accept;
    input.style.display = "none";
    document.body.appendChild(input);
    input.addEventListener("change", () => {
      resolve(input.files ? [...input.files] : null);
      input.remove();
    });
    input.addEventListener("cancel", () => {
      resolve(null);
      input.remove();
    });
    input.click();
  });
}

export async function open(opt: OpenOptions = {}): Promise<string | string[] | null> {
  const title = (opt.title ?? "").toLowerCase();
  if (opt.directory && title.includes("new project")) {
    const name = window.prompt("Name the new project", "My survey");
    if (!name) return null;
    // A unique folder, named so the project takes the name typed.
    return `/w/projects/${Date.now().toString(36)}/${name.replace(/[\\/]/g, "-")}`;
  }
  if (opt.directory && title.includes("open a project")) {
    window.alert("In the web demo, projects live only in this browser tab — pick one from the list, or start a new one.");
    return null;
  }
  if (opt.directory && !title.includes("photo")) {
    // An output folder (e.g. Litchi missions): its files are downloaded.
    return `/w/out/${++outputs}`;
  }
  const accept = opt.filters?.flatMap((f) => f.extensions.map((e) => `.${e}`)).join(",");
  const files = await pick({ directory: opt.directory, multiple: opt.multiple || opt.directory, accept });
  if (!files?.length) return null;
  const photos = opt.directory ? files.filter((f) => isPhoto(f.name)) : files;
  const paths = await addFiles(opt.directory || title.includes("photo") ? "photos" : "in", photos, !!opt.directory);
  if (opt.directory) {
    // The engine walks folders itself; hand it the folder.
    const top = new Set(paths.map((p) => p.split("/").slice(0, 5).join("/")));
    return [...top];
  }
  return opt.multiple ? paths : paths[0];
}

export async function save(opt: SaveOptions = {}): Promise<string | null> {
  const name = (opt.defaultPath ?? "download").split(/[\\/]/).pop()!;
  const path = `/w/out/${++outputs}/${name}`;
  requestDownload(path);
  return path;
}
