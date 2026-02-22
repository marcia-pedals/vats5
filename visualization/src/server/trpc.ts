import { initTRPC } from "@trpc/server";
import { z } from "zod";
import * as fs from "fs/promises";
import * as path from "path";
import { homedir } from "os";
import { VisualizationSchema, type Stop, type Visualization } from "./schemas";

const t = initTRPC.create();

const PROBLEMS_DIR = path.join(homedir(), "vats5-problems");

export const appRouter = t.router({
  // List all *-viz.json files in ~/vats5-problems
  listVisualizations: t.procedure.query(async () => {
    try {
      const files = await fs.readdir(PROBLEMS_DIR);
      const visFiles = files
        .filter((file) => file.endsWith("-viz.json"))
        .map((file) => ({
          filename: file,
          name: file.replace("-viz.json", ""),
        }));
      return visFiles;
    } catch (error) {
      console.error("Error listing visualizations:", error);
      throw new Error("Failed to list visualizations");
    }
  }),

  // Read a specific vis.json file
  getVisualization: t.procedure
    .input(z.object({ filename: z.string() }))
    .query(async ({ input }) => {
      try {
        const filePath = path.join(PROBLEMS_DIR, input.filename);

        // Security check: ensure the file is in the problems directory
        const resolvedPath = path.resolve(filePath);
        const resolvedProblemsDir = path.resolve(PROBLEMS_DIR);
        if (!resolvedPath.startsWith(resolvedProblemsDir)) {
          throw new Error("Invalid file path");
        }

        const content = await fs.readFile(filePath, "utf-8");
        const data = JSON.parse(content);

        // Validate against schema
        const visualization = VisualizationSchema.parse(data);
        return visualization;
      } catch (error) {
        console.error("Error reading visualization:", error);
        throw new Error(`Failed to read visualization: ${error instanceof Error ? error.message : "Unknown error"}`);
      }
    }),
});

export type AppRouter = typeof appRouter;
