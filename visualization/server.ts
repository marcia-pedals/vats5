import { createExpressMiddleware } from "@trpc/server/adapters/express";
import cors from "cors";
import express from "express";
import { CHECKOUT_NAME, TRPC_PORT } from "./ports";
import { appRouter } from "./src/server/trpc";

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
  console.log(
    `tRPC server listening on http://0.0.0.0:${TRPC_PORT} [${CHECKOUT_NAME}]`
  );
});
