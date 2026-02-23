import { tanstackRouter } from "@tanstack/router-plugin/vite";
import tailwindcss from "@tailwindcss/vite";
import react from "@vitejs/plugin-react";
import { defineConfig } from "vite";

const VITE_PORT = Number(process.env.VITE_PORT);
const TRPC_PORT = Number(process.env.TRPC_PORT);

if (!VITE_PORT || !TRPC_PORT) {
  console.error(
    "VITE_PORT / TRPC_PORT not set — use bin/viz to start the server"
  );
  process.exit(1);
}

export default defineConfig({
  plugins: [
    tanstackRouter({ target: "react", autoCodeSplitting: true }),
    tailwindcss(),
    react(),
  ],
  server: {
    host: "0.0.0.0",
    port: VITE_PORT,
    proxy: {
      "/trpc": {
        target: `http://localhost:${TRPC_PORT}`,
        changeOrigin: true,
      },
    },
  },
});
