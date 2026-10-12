// Web demo backend: answers the UI's `invoke()` calls by running the engine
// in a worker (see worker.ts). Everything stays in this browser tab — no
// server, no upload.

type Handler = (e: { payload: any }) => void;

const listeners = new Map<string, Set<Handler>>();

export function on(event: string, cb: Handler) {
  if (!listeners.has(event)) listeners.set(event, new Set());
  listeners.get(event)!.add(cb);
  return () => listeners.get(event)?.delete(cb);
}

function emit(event: string, payload: unknown) {
  listeners.get(event)?.forEach((cb) => cb({ payload }));
}

// ── Worker ──
let worker: Worker;
let seq = 0;
const pending = new Map<number, { resolve: (v: any) => void; reject: (e: Error) => void }>();

function startWorker() {
  worker = new Worker(new URL("./worker.ts", import.meta.url), { type: "module" });
  worker.onmessage = (ev) => {
    const m = ev.data;
    if (m.type === "progress") return emit("progress", m.payload);
    const p = pending.get(m.id);
    if (!p) return;
    pending.delete(m.id);
    m.ok ? p.resolve(m.value) : p.reject(new Error(m.error));
  };
}
startWorker();

function send<T = any>(msg: object, transfer: Transferable[] = []): Promise<T> {
  const id = ++seq;
  return new Promise((resolve, reject) => {
    pending.set(id, { resolve, reject });
    worker.postMessage({ id, ...msg }, transfer);
  });
}

/** Puts the user's files into the demo's file system — as handles on the
 *  originals, not copies. */
export async function ingest(files: { path: string; file: Blob }[]) {
  if (files.length) await send({ type: "ingest", files });
}

export async function readFile(path: string): Promise<Blob | null> {
  const v = await send<Uint8Array | Blob | null>({ type: "read", path });
  if (v == null) return null;
  return v instanceof Blob ? v : new Blob([v as BlobPart]);
}

// ── Projects in this tab ──
const projects = new Map<string, { name: string; photos: number }>();
let job: Promise<unknown> | null = null;

/** Files the user asked to save; downloaded once a command has written them. */
const downloads = new Set<string>();
export function requestDownload(path: string) {
  downloads.add(path);
}

async function flushDownloads() {
  for (const path of [...downloads]) {
    const blob = await readFile(path);
    if (!blob) continue;
    downloads.delete(path);
    saveBlob(blob, path.split("/").pop()!);
    // A world file next to a PNG/JPEG carries its map location.
    const stem = path.replace(/\.[^./]+$/, "");
    for (const ext of [".pgw", ".jgw", ".tfw", ".prj"]) {
      const side = await readFile(stem + ext);
      if (side) saveBlob(side, (stem + ext).split("/").pop()!);
    }
    void send({ type: "remove", path });
  }
}

export function saveBlob(blob: Blob, name: string) {
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = name;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 60_000);
}

const LITE = { kind: "lite", work_size: 1024, max_features: 1500, render_cache_bytes: 320 << 20, threads: 1 };

const JOBS: Record<string, string> = { process: "process", export_map: "export", build_3d: "3d" };

export async function invoke(cmd: string, args: any = {}): Promise<any> {
  switch (cmd) {
    case "system_info":
      return {
        os: "web",
        arch: "wasm32",
        ram_gb: (navigator as any).deviceMemory ?? 4,
        cores: 1,
        profile: LITE,
      };
    case "recent_projects":
      return [...projects].map(([dir, p]) => ({ dir, name: p.name, photos: p.photos }));
    case "cancel_job":
      return cancel();
  }
  if (cmd in JOBS) {
    if (job) throw new Error("another job is already running");
    // Long jobs report through events, exactly like the desktop app.
    job = send({ type: "call", cmd, args })
      .then(async (done) => {
        job = null;
        if (done.ok) await flushDownloads();
        if (done.ok && cmd === "export_map") done.message = "Exported — check your downloads";
        emit("job-finished", done);
      })
      .catch((e: Error) => {
        job = null;
        emit("job-finished", { kind: JOBS[cmd], ok: false, message: e.message, detail: null });
      });
    return null;
  }
  const value = await send({ type: "call", cmd, args });
  if (cmd === "export_litchi" && Array.isArray(value)) {
    for (const path of value as string[]) {
      const blob = await readFile(path);
      if (blob) saveBlob(blob, path.split("/").pop()!);
    }
  }
  if (value && typeof value === "object" && "dir" in value && "photos" in value && Array.isArray(value.photos)) {
    projects.set(value.dir, { name: value.name, photos: value.photos.length });
  }
  if (value?.project?.dir) projects.set(value.project.dir, { name: value.project.name, photos: value.project.photos.length });
  await flushDownloads();
  return value;
}

/** WebAssembly can't be interrupted mid-run, so cancelling restarts the
 *  engine: work done so far in this tab is lost. */
function cancel() {
  if (!job) return;
  worker.terminate();
  // Rejecting the running call ends the job (and reports it) as usual.
  for (const p of pending.values()) p.reject(new Error("Cancelled — the demo restarted, so add your photos again"));
  pending.clear();
  projects.clear();
  startWorker();
  emit("demo-reset", null);
}
