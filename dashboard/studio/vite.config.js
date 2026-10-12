import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// The built app is served by the hub from dashboard/static/studio (commit it: nobody needs Node to run the dashboard).
export default defineConfig({
  base: "/studio/",
  plugins: [react()],
  build: { outDir: "../static/studio", emptyOutDir: true, chunkSizeWarningLimit: 1500 },
  server: { proxy: { "/api": "http://127.0.0.1:8765" } },
});
