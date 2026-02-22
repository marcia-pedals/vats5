import { createExpressMiddleware } from "@trpc/server/adapters/express";
import cors from "cors";
import express from "express";
import { appRouter } from "./src/server/trpc";

const app = express();
const PORT = 4000;

app.use(cors());

app.use(
  "/trpc",
  createExpressMiddleware({
    router: appRouter,
    createContext: () => ({}),
  })
);

app.listen(PORT, "0.0.0.0", () => {
  console.log(`✅ tRPC server listening on http://0.0.0.0:${PORT}`);
});
