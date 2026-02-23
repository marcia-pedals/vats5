import { initTRPC } from "@trpc/server";
import { z } from "zod";
import { getPathSteps, getPaths, getStops, listVisualizations } from "./db";

const t = initTRPC.create();

export const appRouter = t.router({
  listVisualizations: t.procedure.query(() => listVisualizations()),

  getStops: t.procedure.input(z.object({ name: z.string() })).query(({ input }) => {
    return getStops(input.name);
  }),

  getPaths: t.procedure
    .input(z.object({ name: z.string(), origin: z.string(), destination: z.string() }))
    .query(({ input }) => {
      return getPaths(input.name, input.origin, input.destination);
    }),

  getPathSteps: t.procedure
    .input(z.object({ name: z.string(), pathId: z.number() }))
    .query(({ input }) => {
      return getPathSteps(input.name, input.pathId);
    }),
});

export type AppRouter = typeof appRouter;
