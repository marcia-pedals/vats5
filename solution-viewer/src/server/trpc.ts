import { initTRPC } from "@trpc/server";
import { z } from "zod";
import { listRuns, listSolutions } from "./db";

const t = initTRPC.create();

export const appRouter = t.router({
  listRuns: t.procedure.query(() => listRuns()),

  listSolutions: t.procedure
    .input(z.object({ gtfsInstanceId: z.string() }))
    .query(({ input }) => listSolutions(input.gtfsInstanceId)),
});

export type AppRouter = typeof appRouter;
