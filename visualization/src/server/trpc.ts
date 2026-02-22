import { initTRPC } from "@trpc/server";
import { z } from "zod";
import { getPaths, getStops, listVisualizations } from "./db";

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
});

export type AppRouter = typeof appRouter;
