import * as fs from "node:fs/promises";
import { homedir } from "node:os";
import * as path from "node:path";
import Database from "better-sqlite3";
import { z } from "zod";

const PROBLEMS_DIR = path.join(homedir(), "vats5-problems");

function withDb<T>(name: string, callback: (db: Database.Database) => T): T {
  const filePath = path.join(PROBLEMS_DIR, `${name}-viz.sqlite`);
  const resolvedPath = path.resolve(filePath);
  const resolvedProblemsDir = path.resolve(PROBLEMS_DIR);
  if (!resolvedPath.startsWith(resolvedProblemsDir)) {
    throw new Error("Invalid file path");
  }
  const db = new Database(filePath, { readonly: true });
  try {
    return callback(db);
  } finally {
    db.close();
  }
}

// --- Schemas ---

const StopSchema = z.object({
  stop_id: z.string(),
  stop_name: z.string(),
  lat: z.number(),
  lon: z.number(),
  stop_type: z.enum(["required", "in_problem_state", "original"]),
});
export type Stop = z.infer<typeof StopSchema>;

const VizPathRowSchema = z.object({
  path_id: z.number(),
  depart_time: z.number(),
  arrive_time: z.number(),
  is_flex: z.number(),
});
export type VizPath = z.infer<typeof VizPathRowSchema>;

const PathStepRowSchema = z.object({
  origin_stop_id: z.string(),
  destination_stop_id: z.string(),
  depart_time: z.number(),
  arrive_time: z.number(),
  is_flex: z.number(),
  route_direction_id: z.string().nullable(),
});
export type PathStep = z.infer<typeof PathStepRowSchema>;

const RouteSchema = z.object({
  route_direction_id: z.string(),
  route_name: z.string(),
  route_color: z.string(),
  route_text_color: z.string(),
});
export type Route = z.infer<typeof RouteSchema>;

const PartialSolutionRunSchema = z.object({
  run_timestamp: z.string(),
  max_iteration: z.number(),
});
export type PartialSolutionRun = z.infer<typeof PartialSolutionRunSchema>;

const PartialSolutionPathSchema = z.object({
  steps: z.array(PathStepRowSchema), // Collapsed steps (grouped by trip)
  original_steps: z.array(PathStepRowSchema), // Original uncollapsed steps
  duration: z.number(),
});

const PartialSolutionDataSchema = z.object({
  leaves: z.array(z.string()),
  paths: z.array(PartialSolutionPathSchema),
  best_path: PartialSolutionPathSchema,
});
export type PartialSolutionData = z.infer<typeof PartialSolutionDataSchema>;

// --- Queries ---

export async function listVisualizations(): Promise<{ filename: string; name: string }[]> {
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
}

export function getStops(name: string): Stop[] {
  return withDb(name, (db) => {
    const rows = db.prepare("SELECT stop_id, stop_name, lat, lon, stop_type FROM stops").all();
    return z.array(StopSchema).parse(rows);
  });
}

export function getPaths(name: string, origin: string, destination: string): VizPath[] {
  return withDb(name, (db) => {
    const rows = db
      .prepare(
        "SELECT path_id, depart_time, arrive_time, is_flex FROM paths WHERE origin_stop_id = ? AND destination_stop_id = ?"
      )
      .all(origin, destination);
    return z.array(VizPathRowSchema).parse(rows);
  });
}

export function getPathSteps(name: string, pathId: number): PathStep[] {
  return withDb(name, (db) => {
    const rows = db
      .prepare(
        "SELECT origin_stop_id, destination_stop_id, depart_time, arrive_time, is_flex, route_direction_id FROM paths_steps WHERE path_id = ?"
      )
      .all(pathId);
    return z.array(PathStepRowSchema).parse(rows);
  });
}

export function getRoutes(name: string): Route[] {
  return withDb(name, (db) => {
    const rows = db
      .prepare("SELECT route_direction_id, route_name, route_color, route_text_color FROM routes")
      .all();
    return z.array(RouteSchema).parse(rows);
  });
}

export function getPartialSolutionRuns(name: string): PartialSolutionRun[] {
  return withDb(name, (db) => {
    const tableExists = db
      .prepare("SELECT name FROM sqlite_master WHERE type='table' AND name='partial_solutions'")
      .get();
    if (!tableExists) return [];
    const rows = db
      .prepare(
        "SELECT run_timestamp, MAX(iteration) as max_iteration FROM partial_solutions GROUP BY run_timestamp ORDER BY run_timestamp DESC"
      )
      .all();
    return z.array(PartialSolutionRunSchema).parse(rows);
  });
}

export function getPartialSolution(
  name: string,
  runTimestamp: string,
  iteration: number
): PartialSolutionData | null {
  return withDb(name, (db) => {
    const row = db
      .prepare("SELECT data FROM partial_solutions WHERE run_timestamp = ? AND iteration = ?")
      .get(runTimestamp, iteration) as { data: string } | undefined;
    if (!row) return null;
    return PartialSolutionDataSchema.parse(JSON.parse(row.data));
  });
}
