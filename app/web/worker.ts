// Web demo engine: runs the same Rust engine as the desktop app, compiled to
// WebAssembly (WASI), against an in-memory file system that lives in this
// worker. Photos are NOT copied into memory: each one is a handle on the
// user's own file, read in slices on demand (FileReaderSync).
//
// Messages in:  { id, type: "call", cmd, args }      → { id, ok, value | error }
//               { id, type: "ingest", files: [{ path, file }] }
//               { id, type: "read", path }           → bytes or null
//               { id, type: "remove", path }
// Messages out: { type: "progress", payload }        while a call runs

import { WASI, File, Directory, OpenFile, ConsoleStdout, PreopenDirectory, Inode, Fd, wasi } from "@bjorn3/browser_wasi_shim";

declare const self: DedicatedWorkerGlobalScope;
declare const FileReaderSync: { new (): { readAsArrayBuffer(b: Blob): ArrayBuffer } };

// ── Faster in-memory writes ──
// The shim reallocates the whole file on every write, so writing a 50 MB
// GeoTIFF in 8 KB chunks would copy ~150 GB. Grow geometrically instead.
function grow(file: File, end: number) {
  const data = file.data;
  if (end <= data.byteLength) return;
  if (data.byteOffset === 0 && end <= data.buffer.byteLength) {
    file.data = new Uint8Array(data.buffer, 0, end);
    return;
  }
  const cap = Math.max(end, Math.ceil(data.byteLength * 1.5), 64 * 1024);
  const next = new Uint8Array(new ArrayBuffer(cap), 0, end);
  next.set(data);
  file.data = next;
}
const proto = OpenFile.prototype as any;
proto.fd_write = function (this: any, data: Uint8Array) {
  if (this.file.readonly) return { ret: wasi.ERRNO_BADF, nwritten: 0 };
  const pos = Number(this.file_pos);
  grow(this.file, pos + data.byteLength);
  this.file.data.set(data, pos);
  this.file_pos += BigInt(data.byteLength);
  return { ret: 0, nwritten: data.byteLength };
};
proto.fd_pwrite = function (this: any, data: Uint8Array, offset: bigint) {
  if (this.file.readonly) return { ret: wasi.ERRNO_BADF, nwritten: 0 };
  grow(this.file, Number(offset) + data.byteLength);
  this.file.data.set(data, Number(offset));
  return { ret: 0, nwritten: data.byteLength };
};

// ── Photos: read straight from the user's file, never copied ──
const reader = new FileReaderSync();

class BlobFile extends Inode {
  constructor(readonly blob: Blob) {
    super();
  }
  path_open(_oflags: number, fs_rights_base: bigint) {
    if ((fs_rights_base & BigInt(wasi.RIGHTS_FD_WRITE)) === BigInt(wasi.RIGHTS_FD_WRITE)) {
      return { ret: wasi.ERRNO_PERM, fd_obj: null };
    }
    return { ret: wasi.ERRNO_SUCCESS, fd_obj: new OpenBlob(this) };
  }
  get size() {
    return BigInt(this.blob.size);
  }
  stat() {
    return new wasi.Filestat(this.ino, wasi.FILETYPE_REGULAR_FILE, this.size);
  }
}

class OpenBlob extends Fd {
  pos = 0;
  constructor(readonly f: BlobFile) {
    super();
  }
  private slice(at: number, size: number) {
    const end = Math.min(this.f.blob.size, at + size);
    if (end <= at) return new Uint8Array(0);
    return new Uint8Array(reader.readAsArrayBuffer(this.f.blob.slice(at, end)));
  }
  fd_read(size: number) {
    const data = this.slice(this.pos, size);
    this.pos += data.byteLength;
    return { ret: 0, data };
  }
  fd_pread(size: number, offset: bigint) {
    return { ret: 0, data: this.slice(Number(offset), size) };
  }
  fd_seek(offset: bigint, whence: number) {
    const base = whence === wasi.WHENCE_SET ? 0 : whence === wasi.WHENCE_CUR ? this.pos : this.f.blob.size;
    const p = base + Number(offset);
    if (p < 0) return { ret: wasi.ERRNO_INVAL, offset: 0n };
    this.pos = p;
    return { ret: 0, offset: BigInt(p) };
  }
  fd_tell() {
    return { ret: 0, offset: BigInt(this.pos) };
  }
  fd_filestat_get() {
    return { ret: 0, filestat: this.f.stat() };
  }
  fd_fdstat_get() {
    return { ret: 0, fdstat: new wasi.Fdstat(wasi.FILETYPE_REGULAR_FILE, 0) };
  }
}

// ── File system ──
const root = new PreopenDirectory(
  "/w",
  new Map<string, Inode>([
    ["photos", new Directory([])],
    ["projects", new Directory([])],
    ["out", new Directory([])],
  ]),
);

function parts(path: string) {
  if (!path.startsWith("/w/")) throw new Error(`outside the demo's file system: ${path}`);
  return path.slice(3).split("/").filter(Boolean);
}

function lookup(path: string): Inode | null {
  let cur: Inode = root.dir;
  for (const p of parts(path)) {
    if (!(cur instanceof Directory)) return null;
    const next = cur.contents.get(p);
    if (!next) return null;
    cur = next;
  }
  return cur;
}

function mkdirs(names: string[]): Directory {
  let cur = root.dir;
  for (const n of names) {
    let next = cur.contents.get(n);
    if (!next) {
      next = new Directory([]);
      (next as any).parent = cur;
      cur.contents.set(n, next);
    }
    if (!(next instanceof Directory)) throw new Error(`${n} is not a folder`);
    cur = next;
  }
  return cur;
}

// ── Engine ──
let module: WebAssembly.Module | null = null;
async function engine() {
  if (!module) {
    const url = new URL(`${import.meta.env.BASE_URL}aerial-stitch.wasm`, self.location.origin);
    module = await WebAssembly.compileStreaming(fetch(url));
  }
  return module;
}

async function call(cmd: string, args: unknown) {
  const stdin = new TextEncoder().encode(JSON.stringify({ cmd, args }));
  let stdout = "";
  const errors: string[] = [];
  const fds = [
    new OpenFile(new File(stdin)),
    new ConsoleStdout((b: Uint8Array) => (stdout += new TextDecoder().decode(b))),
    ConsoleStdout.lineBuffered((line: string) => {
      if (line.startsWith("@progress ")) {
        self.postMessage({ type: "progress", payload: JSON.parse(line.slice(10)) });
      } else if (line.trim()) {
        errors.push(line);
      }
    }),
    root,
  ];
  const w = new WASI(["aerial-stitch", "api"], [], fds, { debug: false });
  const inst = await WebAssembly.instantiate(await engine(), { wasi_snapshot_preview1: w.wasiImport });
  try {
    w.start(inst as any);
  } catch (e) {
    // A Rust panic or running out of memory lands here.
    const msg = String(e);
    throw new Error(
      /memory|allocation|RangeError/i.test(msg + errors.join(" "))
        ? "Ran out of memory in the browser. Try fewer photos, a lower output detail, or the desktop app."
        : `${errors.slice(-3).join(" ") || msg}`,
    );
  }
  const line = stdout.trim().split("\n").pop() || "";
  if (!line) throw new Error(errors.slice(-3).join(" ") || "the engine stopped without an answer");
  const r = JSON.parse(line);
  if (!r.ok) throw new Error(r.error);
  return r.value;
}

self.onmessage = async (ev: MessageEvent) => {
  const m = ev.data;
  try {
    switch (m.type) {
      case "call":
        self.postMessage({ id: m.id, ok: true, value: await call(m.cmd, m.args) });
        return;
      case "ingest": {
        for (const { path, file } of m.files as { path: string; file: Blob }[]) {
          const ps = parts(path);
          const dir = mkdirs(ps.slice(0, -1));
          dir.contents.set(ps[ps.length - 1], new BlobFile(file));
        }
        self.postMessage({ id: m.id, ok: true, value: null });
        return;
      }
      case "read": {
        const f = lookup(m.path);
        if (f instanceof File) {
          const copy = f.data.slice();
          self.postMessage({ id: m.id, ok: true, value: copy }, [copy.buffer]);
        } else if (f instanceof BlobFile) {
          self.postMessage({ id: m.id, ok: true, value: f.blob });
        } else {
          self.postMessage({ id: m.id, ok: true, value: null });
        }
        return;
      }
      case "remove": {
        const ps = parts(m.path);
        const dir = lookup("/w/" + ps.slice(0, -1).join("/"));
        if (dir instanceof Directory) dir.contents.delete(ps[ps.length - 1]);
        self.postMessage({ id: m.id, ok: true, value: null });
        return;
      }
    }
  } catch (e) {
    self.postMessage({ id: m.id, ok: false, error: e instanceof Error ? e.message : String(e) });
  }
};
