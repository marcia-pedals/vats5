import { initTRPC } from "@trpc/server";
import { z } from "zod";
import { listRuns, listSolutions, listTargetStops } from "./db.ts";

const t = initTRPC.create();

export const appRouter = t.router({
  listTargetStops: t.procedure.query(() => listTargetStops()),

  listRuns: t.procedure
    .input(z.object({ targetStopsId: z.string() }))
    .query(({ input }) => listRuns(input.targetStopsId)),

  listSolutions: t.procedure
    .input(z.object({ gtfsInstanceId: z.string(), targetStopsId: z.string() }))
    .query(({ input }) => listSolutions(input.gtfsInstanceId, input.targetStopsId)),
});

export type AppRouter = typeof appRouter;
