import * as fs from "node:fs/promises";
import { homedir } from "node:os";
import * as path from "node:path";
import { initTRPC } from "@trpc/server";
import Database from "better-sqlite3";
import { z } from "zod";
import type { Stop, VizPath } from "./schemas";

const t = initTRPC.create();

const PROBLEMS_DIR = path.join(homedir(), "vats5-problems");

export const appRouter = t.router({
  // List all *-viz.sqlite files in ~/vats5-problems
  listVisualizations: t.procedure.query(async () => {
    try {
      const files = await fs.readdir(PROBLEMS_DIR);
      const visFiles = files
        .filter((file) => file.endsWith("-viz.sqlite"))
        .map((file) => ({
          filename: file,
          name: file.replace("-viz.sqlite", ""),
        }));
      return visFiles;
    } catch (error) {
      console.error("Error listing visualizations:", error);
      throw new Error("Failed to list visualizations");
    }
  }),

  // Get all stops from a visualization
  getStops: t.procedure.input(z.object({ name: z.string() })).query(({ input }) => {
    const filePath = path.join(PROBLEMS_DIR, `${input.name}-viz.sqlite`);
    const resolvedPath = path.resolve(filePath);
    const resolvedProblemsDir = path.resolve(PROBLEMS_DIR);
    if (!resolvedPath.startsWith(resolvedProblemsDir)) {
      throw new Error("Invalid file path");
    }

    const db = new Database(filePath, { readonly: true });
    try {
      const stops = db.prepare("SELECT stop_id, stop_name, lat, lon FROM stops").all() as Stop[];
      return stops;
    } finally {
      db.close();
    }
  }),

  // Get paths between a specific origin/destination pair
  getPaths: t.procedure
    .input(
      z.object({
        name: z.string(),
        origin: z.string(),
        destination: z.string(),
      })
    )
    .query(({ input }) => {
      const filePath = path.join(PROBLEMS_DIR, `${input.name}-viz.sqlite`);
      const resolvedPath = path.resolve(filePath);
      const resolvedProblemsDir = path.resolve(PROBLEMS_DIR);
      if (!resolvedPath.startsWith(resolvedProblemsDir)) {
        throw new Error("Invalid file path");
      }

      const db = new Database(filePath, { readonly: true });
      try {
        const rows = db
          .prepare(
            "SELECT depart_time, arrive_time, duration_seconds, is_flex FROM paths WHERE origin_stop_id = ? AND destination_stop_id = ?"
          )
          .all(input.origin, input.destination) as Array<{
          depart_time: number;
          arrive_time: number;
          duration_seconds: number;
          is_flex: number;
        }>;
        // Convert is_flex from 0/1 integer to boolean
        return rows.map(
          (r): VizPath => ({
            depart_time: r.depart_time,
            arrive_time: r.arrive_time,
            duration_seconds: r.duration_seconds,
            is_flex: r.is_flex === 1,
          })
        );
      } finally {
        db.close();
      }
    }),
});

export type AppRouter = typeof appRouter;
