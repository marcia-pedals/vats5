import { initTRPC } from "@trpc/server";
import { listHelloWorld } from "./db";

const t = initTRPC.create();

export const appRouter = t.router({
  listHelloWorld: t.procedure.query(() => listHelloWorld()),
});

export type AppRouter = typeof appRouter;
