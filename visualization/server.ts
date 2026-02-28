import { createExpressMiddleware } from "@trpc/server/adapters/express";
import cors from "cors";
import express from "express";
import { appRouter } from "./src/server/trpc";

const TRPC_PORT = Number(process.env.TRPC_PORT);
const CHECKOUT_NAME = process.env.CHECKOUT_NAME ?? "unknown";

if (!TRPC_PORT) {
  console.error("TRPC_PORT not set — use bin/viz to start the server");
  process.exit(1);
}

const app = express();

app.use(cors());

app.use(
  "/trpc",
  createExpressMiddleware({
    router: appRouter,
    createContext: () => ({}),
  })
);

app.listen(TRPC_PORT, "0.0.0.0", () => {
  console.log(`tRPC server listening on http://0.0.0.0:${TRPC_PORT} [${CHECKOUT_NAME}]`);
});
