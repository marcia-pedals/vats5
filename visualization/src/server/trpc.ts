import * as fs from "node:fs/promises";
import { homedir } from "node:os";
import * as path from "node:path";
import { initTRPC } from "@trpc/server";
import { z } from "zod";
import { getPaths, getStops } from "./db";

const t = initTRPC.create();

const PROBLEMS_DIR = path.join(homedir(), "vats5-problems");

export const appRouter = t.router({
  listVisualizations: t.procedure.query(async () => {
    try {
      const files = await fs.readdir(PROBLEMS_DIR);
      return files
        .filter((file) => file.endsWith("-viz.sqlite"))
        .map((file) => ({
          filename: file,
          name: file.replace("-viz.sqlite", ""),
        }));
    } catch (error) {
      console.error("Error listing visualizations:", error);
      throw new Error("Failed to list visualizations");
    }
  }),

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
