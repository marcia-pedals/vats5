import { tanstackRouter } from "@tanstack/router-plugin/vite";
import tailwindcss from "@tailwindcss/vite";
import react from "@vitejs/plugin-react";
import { defineConfig } from "vite";

const VITE_PORT = Number(process.env.VITE_PORT);
const TRPC_PORT = Number(process.env.TRPC_PORT);
const CHECKOUT_NAME = process.env.CHECKOUT_NAME;

if (!VITE_PORT || !TRPC_PORT || !CHECKOUT_NAME) {
  console.error(
    "VITE_PORT / TRPC_PORT / CHECKOUT_NAME not set — use bin/viz to start the server"
  );
  process.exit(1);
}

export default defineConfig({
  define: {
    __CHECKOUT_NAME__: JSON.stringify(CHECKOUT_NAME),
  },
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
