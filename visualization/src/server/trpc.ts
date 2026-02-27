import { initTRPC } from "@trpc/server";
import { z } from "zod";
import {
  getPartialSolution,
  getPartialSolutionRuns,
  getPathSteps,
  getPaths,
  getRoutes,
  getStops,
  listVisualizations,
} from "./db";

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

  getRoutes: t.procedure.input(z.object({ name: z.string() })).query(({ input }) => {
    return getRoutes(input.name);
  }),

  getPartialSolutionRuns: t.procedure.input(z.object({ name: z.string() })).query(({ input }) => {
    return getPartialSolutionRuns(input.name);
  }),

  getPartialSolution: t.procedure
    .input(z.object({ name: z.string(), runTimestamp: z.string(), iteration: z.number() }))
    .query(({ input }) => {
      return getPartialSolution(input.name, input.runTimestamp, input.iteration);
    }),
});

export type AppRouter = typeof appRouter;
