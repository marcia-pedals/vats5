import { tanstackRouter } from "@tanstack/router-plugin/vite";
import tailwindcss from "@tailwindcss/vite";
import react from "@vitejs/plugin-react";
import { defineConfig } from "vite";
import { CHECKOUT_NAME, TRPC_PORT, VITE_PORT } from "./ports";

export default defineConfig({
  plugins: [
    tanstackRouter({ target: "react", autoCodeSplitting: true }),
    tailwindcss(),
    react(),
    {
      name: "log-checkout-ports",
      configResolved() {
        console.log(
          `\n  checkout: ${CHECKOUT_NAME}\n  vite:    http://localhost:${VITE_PORT}\n  trpc:    http://localhost:${TRPC_PORT}\n`
        );
      },
    },
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
