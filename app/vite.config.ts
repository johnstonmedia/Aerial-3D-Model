import { defineConfig } from "vite";
import { fileURLToPath } from "node:url";

const here = (p: string) => fileURLToPath(new URL(p, import.meta.url));

// `vite build --mode web` builds the browser demo: the same UI, with the
// Tauri APIs swapped for stand-ins that run the engine as WebAssembly.
export default defineConfig(({ mode }) => {
  const web = mode === "web";
  return {
    clearScreen: false,
    // Tauri expects a fixed port; GitHub Pages serves the demo from /<repo>/.
    base: web ? process.env.DEMO_BASE ?? "/" : "/",
    server: { port: 5173, strictPort: true },
    resolve: {
      alias: web
        ? {
            "@platform": here("./web/platform.ts"),
            "@tauri-apps/api/core": here("./web/tauri/core.ts"),
            "@tauri-apps/api/event": here("./web/tauri/event.ts"),
            "@tauri-apps/api/webview": here("./web/tauri/webview.ts"),
            "@tauri-apps/plugin-dialog": here("./web/tauri/dialog.ts"),
          }
        : { "@platform": here("./src/platform.ts") },
    },
    // The engine (aerial-stitch.wasm) is copied in by the build, not committed.
    publicDir: web ? "public-web" : "public",
    worker: { format: "es" },
    build: { target: "es2021", outDir: web ? "dist-web" : "dist", emptyOutDir: true },
  };
});
