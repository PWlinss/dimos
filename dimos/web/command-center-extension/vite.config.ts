import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import { resolve } from "path";

export default defineConfig({
  plugins: [react()],
  root: ".",
  build: {
    outDir: "dist-standalone",
    emptyDirBeforeWrite: true,
    rollupOptions: {
      input: {
        main: resolve(__dirname, "index.html"),
      },
    },
  },
  server: {
    port: 3000,
    open: false,
    proxy: {
      "/socket.io": {
        target: "http://127.0.0.1:7779",
        ws: true,
      },
    },
  },
});
