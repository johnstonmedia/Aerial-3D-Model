import { defineConfig } from "vite";

// Tauri expects a fixed port and doesn't want Vite clearing its output.
export default defineConfig({
  clearScreen: false,
  server: { port: 5173, strictPort: true },
  build: { target: "es2021", outDir: "dist", emptyOutDir: true },
});
